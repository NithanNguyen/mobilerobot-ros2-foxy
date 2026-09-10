#!/usr/bin/env python3
# launch/slam_launch.py
# ──────────────────────────────────────────────────────────────────────────────
# Chạy trên JETSON AGX Xavier — Full Hardware Bringup + EKF + SLAM Toolbox
#
# Stack:
#   1. robot_state_publisher      — URDF + TF tĩnh                    [t = 0.0s]
#   2. joint_state_publisher      — /joint_states                     [t = 1.5s]
#   3. wheel_odom_node            — STM32 UART → /odom                [t = 0.0s]
#   4. bno055                     — I2C → /imu/data (~100 Hz)         [t = 0.0s]
#   5. imu_reader                 — Quaternion → /imu/euler           [t = 3.0s]
#   6. ekf_filter_node            — /odom + /imu/data → TF odom→base  [t = 3.0s]
#   7. sllidar_node               — RPLIDAR S2E UDP → /scan           [t = 0.0s]
#   8. scan_to_scan_filter_chain  — /scan → /scan_filtered            [t = 10.0s]
#   9. async_slam_toolbox_node    — Online Async SLAM → /map          [t = 12.0s]
#
# TF tree:
#   map ←(SLAM)← odom ←(EKF)← base_footprint ← base_link
#                                              ├── chassis ── laser_frame
#                                              │           ── imu_link
#                                              │           ── caster_wheel
#                                              ├── left_wheel
#                                              └── right_wheel
#
# Môi trường (Jetson + Laptop phải giống nhau):
#   export ROS_DOMAIN_ID=42
#   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#
# Chạy:
#   ros2 launch mobile_robot slam_launch.py
#
# Tiếp tục từ map đã lưu (.posegraph):
#   ros2 launch mobile_robot slam_launch.py map_file_name:=/home/<user>/maps/my_map
#
# Lưu map:
#   # Lưu ảnh PNG/PGM + YAML (dùng cho Nav2):
#   ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
#   # Lưu .posegraph (tiếp tục SLAM sau này):
#   ros2 service call /slam_toolbox/serialize_map \
#       slam_toolbox/srv/SerializePoseGraph \
#       "{filename: {data: '/home/<user>/maps/my_map'}}"
#
# Kiểm tra:
#   ros2 topic hz /scan_filtered   # ~10 Hz
#   ros2 topic hz /map             # cập nhật mỗi 5s
#   ros2 topic echo /tf --once     # phải thấy map→odom và odom→base_footprint
#   ros2 run tf2_tools view_frames # export TF tree PDF
# ──────────────────────────────────────────────────────────────────────────────

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # ── Package & shared paths ─────────────────────────────────────────────────
    package_name = 'mobile_robot'
    pkg_share    = get_package_share_directory(package_name)

    urdf_file         = os.path.join(pkg_share, 'urdf',   'mobile_robot.urdf.xacro')
    ekf_config        = os.path.join(pkg_share, 'config', 'ekf.yaml')
    bno055_config     = os.path.join(pkg_share, 'config', 'bno055_params.yaml')
    wheel_odom_config = os.path.join(pkg_share, 'config', 'wheel_odom_params.yaml')
    filter_config     = os.path.join(pkg_share, 'config', 'laser_filter.yaml')
    slam_config       = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

    # ── Launch arguments ───────────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_file = DeclareLaunchArgument(
        'map_file_name',
        default_value='',
        description='Full path (no extension) to .posegraph to resume. '
                    'Leave empty to start a new map.'
    )

    use_sim_time  = LaunchConfiguration('use_sim_time')
    map_file_name = LaunchConfiguration('map_file_name')

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 1 — Robot State Publisher                                    [t = 0s]
    # ══════════════════════════════════════════════════════════════════════════
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time':      use_sim_time,
            'robot_description': Command(['xacro ', urdf_file]),
            'publish_frequency': 10.0,
        }]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 2 — Joint State Publisher                                  [t = 1.5s]
    # ══════════════════════════════════════════════════════════════════════════
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    delayed_jsp = TimerAction(
        period=1.5,
        actions=[
            LogInfo(msg='[slam] [1.5s] Starting joint_state_publisher...'),
            joint_state_publisher,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 3 — Wheel Odometry Node (STM32 via UART)                    [t = 0s]
    # ══════════════════════════════════════════════════════════════════════════
    # publish_tf = false (wheel_odom_params.yaml) — EKF lo TF odom→base_footprint
    wheel_odom_node = Node(
        package='mobile_robot',
        executable='wheel_odom_node',
        name='wheel_odom_node',
        output='screen',
        parameters=[wheel_odom_config],
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 4 — BNO055 IMU Driver (I2C bus 8, J23)                      [t = 0s]
    # ══════════════════════════════════════════════════════════════════════════
    # KHÔNG dùng inline params — tránh override yaml làm mất ros_topic_prefix.
    # remapping: /imu/imu → /imu/data để EKF nhận đúng topic.
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        parameters=[bno055_config],
        remappings=[
            ('/imu/imu', '/imu/data'),
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 5 — IMU Reader (Quaternion → Euler)                        [t = 3.0s]
    # ══════════════════════════════════════════════════════════════════════════
    imu_reader_node = Node(
        package='mobile_robot',
        executable='imu_reader',
        name='imu_reader',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    delayed_imu_reader = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[slam] [3.0s] Starting imu_reader...'),
            imu_reader_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 6 — EKF (robot_localization)                               [t = 3.0s]
    # ══════════════════════════════════════════════════════════════════════════
    # Fuse /odom + /imu/data → publish TF odom → base_footprint
    # SLAM Toolbox sẽ publish TF map → odom (KHÔNG dùng static_tf ở đây!)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )

    delayed_ekf = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg='[slam] [7.0s] Starting ekf_filter_node...'),
            ekf_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 7 — RPLIDAR S2E Driver (UDP)                                [t = 0s]
    # ══════════════════════════════════════════════════════════════════════════
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type':     'udp',
            'udp_ip':           '192.168.11.2',
            'udp_port':         8089,
            'frame_id':         'laser_frame',
            'inverted':         False,
            'angle_compensate': True,
            'scan_mode':        'Sensitivity',
            'use_sim_time':     use_sim_time,
        }]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 8 — Laser Scan Filter                                      [t = 10s]
    # ══════════════════════════════════════════════════════════════════════════
    # Chờ TF chain đầy đủ: odom → base_footprint → … → laser_frame
    # filter1 (box_filter):   loại tự phát hiện thân robot
    # filter2 (range_filter): loại điểm < 0.06m và > 30m
    # Output: /scan_filtered → đây là topic SLAM Toolbox subscribe
    scan_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[filter_config, {'use_sim_time': use_sim_time}]
    )

    delayed_scan_filter = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[slam] [10.0s] Starting scan_to_scan_filter_chain...'),
            scan_filter_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 9 — SLAM Toolbox (Online Async)                            [t = 12s]
    # ══════════════════════════════════════════════════════════════════════════
    # Delay 12s = 10s (chờ scan filter) + 2s buffer cho /scan_filtered ổn định.
    #
    # Cấu hình đọc từ mapper_params_online_async.yaml:
    #   scan_topic:  /scan_filtered  ← PHẢI khớp output của NODE 8
    #   odom_frame:  odom
    #   base_frame:  base_footprint
    #   map_frame:   map
    #   mode:        mapping
    #
    # Publish:
    #   /map          nav_msgs/OccupancyGrid  (mỗi map_update_interval = 5.0s)
    #   TF: map → odom
    #
    # Khi map_file_name trỗng (""):  bắt đầu map mới.
    # Khi có giá trị: deserialize .posegraph và tiếp tục từ đó.
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {
                'use_sim_time':  use_sim_time,
                'map_file_name': map_file_name,
            }
        ],
    )

    delayed_slam = TimerAction(
        period=18.0,
        actions=[
            LogInfo(msg='[slam] [18.0s] Starting async_slam_toolbox_node...'),
            slam_toolbox_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # Launch Description
    # ══════════════════════════════════════════════════════════════════════════
    return LaunchDescription([
        declare_use_sim_time,
        declare_map_file,

        # ── t = 0.0s ────────────────────────────────────────────────────────
        LogInfo(msg='[slam] ═══ Starting Full Hardware Bringup + EKF + SLAM ═══'),
        LogInfo(msg='[slam] [0.0s] Starting RSP, wheel_odom, bno055, lidar...'),
        robot_state_publisher,
        wheel_odom_node,
        bno055_node,
        lidar_node,

        # ── t = 1.5s ────────────────────────────────────────────────────────
        delayed_jsp,

        # ── t = 7.0s ────────────────────────────────────────────────────────
        delayed_ekf,

        # ── t = 3.0s ────────────────────────────────────────────────────────
        delayed_imu_reader,

        # ── t = 10.0s ───────────────────────────────────────────────────────
        delayed_scan_filter,

        # ── t = 18.0s ───────────────────────────────────────────────────────
        delayed_slam,
    ])