#!/usr/bin/env python3
# launch/view_robot_launch.py
# ──────────────────────────────────────────────────────────────────────────────
# Chạy trên JETSON AGX Xavier — Full Hardware Bringup (Real Robot)
#
# Stack bao gồm:
#   1. robot_state_publisher      — URDF + TF tĩnh
#   2. joint_state_publisher      — /joint_states (delay 1.5s)
#   3. wheel_odom_node            — Đọc UART STM32 → /odom
#   4. bno055                     — IMU driver I2C → /imu/data (~100 Hz)
#   5. imu_reader                 — Quaternion → Euler, /imu/euler (delay 3.0s)
#   6. ekf_filter_node            — Fuse /odom + /imu/data → TF odom→base_footprint (delay 3.0s)
#   7. sllidar_node               — RPLIDAR S2E UDP → /scan
#   8. scan_to_scan_filter_chain  — /scan → /scan_filtered (delay 10.0s)
#
# TF tree khi đầy đủ:
#   map ← (SLAM) ← odom ← (EKF) ← base_footprint ← base_link
#                                                   ├── chassis ── laser_frame
#                                                   │           ── imu_link
#                                                   │           ── caster_wheel
#                                                   ├── left_wheel
#                                                   └── right_wheel
#
# Topics chính:
#   /odom                 nav_msgs/Odometry        (wheel_odom_node, ~100 Hz)
#   /imu/data             sensor_msgs/Imu          (bno055, ~100 Hz)
#   /imu/euler            geometry_msgs/Vector3    (imu_reader, rad)
#   /scan                 sensor_msgs/LaserScan    (sllidar_node, 10 Hz)
#   /scan_filtered        sensor_msgs/LaserScan    (laser_filter, 10 Hz)
#   /robot_description    std_msgs/String          (robot_state_publisher)
#   /tf, /tf_static       tf2_msgs/TFMessage
#
# Thứ tự khởi động (được thiết kế để tránh race condition):
#   0.0s  → robot_state_publisher, wheel_odom_node, bno055, sllidar_node
#   1.5s  → joint_state_publisher   (chờ /robot_description sẵn sàng)
#   3.0s  → ekf_filter_node         (chờ /odom và /imu/data bắt đầu publish)
#   3.0s  → imu_reader              (chờ BNO055 khởi động xong)
#   10.0s → scan_to_scan_filter_chain (chờ TF chain đầy đủ: odom→…→laser_frame)
#
# Yêu cầu môi trường (cả Jetson lẫn Laptop — PHẢI GIỐNG NHAU):
#   export ROS_DOMAIN_ID=42
#   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#
# Nếu WiFi chặn multicast (router không forward UDP broadcast):
#   # Trên Jetson — khởi động discovery server:
#   fastdds discovery --server-id 0 --ip-address <IP_JETSON> --port 11811
#   # Trên cả hai máy, thêm:
#   export ROS_DISCOVERY_SERVER=<IP_JETSON>:11811
#
# Chạy trên Jetson (qua SSH):
#   ros2 launch mobile_robot view_robot_launch.py
#
# Chạy RViz trên Laptop:
#   ros2 launch mobile_robot rviz_launch.py
#
# Kiểm tra nhanh:
#   ros2 topic hz /scan                # kỳ vọng ~10 Hz
#   ros2 topic hz /imu/data            # kỳ vọng ~100 Hz
#   ros2 topic hz /odom                # kỳ vọng ~100 Hz
#   ros2 topic echo /tf --once         # phải thấy odom→base_footprint từ EKF
#   ros2 run tf2_tools view_frames     # export TF tree ra file PDF
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

    # ── Launch arguments ───────────────────────────────────────────────────────
    # Luôn False cho robot thật (True chỉ dùng khi Gazebo simulation)
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 1 — Robot State Publisher                                    [t = 0s]
    # ══════════════════════════════════════════════════════════════════════════
    # Publish /robot_description từ URDF xacro và các TF tĩnh:
    #   base_footprint → base_link → chassis → laser_frame
    #                                        → imu_link
    #                                        → caster_wheel
    #              base_link → left_wheel
    #              base_link → right_wheel
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time':      use_sim_time,
            'robot_description': Command(['xacro ', urdf_file]),
            'publish_frequency': 30.0,  # Hz — đủ để Laptop nhận qua WiFi
        }]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 2 — Joint State Publisher                                  [t = 1.5s]
    # ══════════════════════════════════════════════════════════════════════════
    # Publish /joint_states cho left_wheel_joint và right_wheel_joint.
    # Delay 1.5s để đảm bảo /robot_description đã được publish trước.
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
            LogInfo(msg='[view_robot] [1.5s] Starting joint_state_publisher...'),
            joint_state_publisher,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 3 — Wheel Odometry Node (STM32 via UART)                    [t = 0s]
    # ══════════════════════════════════════════════════════════════════════════
    # Đọc packet từ STM32F103 qua USB-TTL (/dev/ttyUSB0).
    # Tính toán odometry từ vận tốc góc bánh trái/phải và publish /odom.
    # publish_tf = false vì EKF sẽ publish TF odom → base_footprint.
    #
    # Packet format (8 bytes, little-endian):
    #   [0x CD] [0xAB] [wL_i16] [wR_i16] [checksum_u16]
    #   omega (rad/s) = raw_value / 100.0
    wheel_odom_node = Node(
        package='mobile_robot',
        executable='wheel_odom_node',
        name='wheel_odom_node',
        output='screen',
        parameters=[wheel_odom_config],
        # Nếu gặp lỗi permission: sudo chmod 666 /dev/ttyUSB0
        # hoặc: sudo usermod -aG dialout $USER (rồi logout/login lại)
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 4 — BNO055 IMU Driver (I2C)                                 [t = 0s]
    # ══════════════════════════════════════════════════════════════════════════
    # Kết nối qua I2C bus 8 (/dev/i2c-8) trên cổng J23 của X221-AI.
    # Load từ bno055_params.yaml để đảm bảo:
    #   - ros_topic_prefix = "imu/"  → publish /imu/data, /imu/mag, /imu/temp
    #   - operation_mode = 0x08      → IMU mode (fusion không có Magnetometer)
    #   - frame_id = "imu_link"
    #   - data_query_frequency = 100 Hz
    #
    # QUAN TRỌNG: Không dùng inline params ở đây vì sẽ override yaml và
    # ros_topic_prefix sẽ về default "bno055/" → EKF không nhận /imu/data.
    #
    # Nếu gặp lỗi Permission denied trên /dev/i2c-8:
    #   sudo usermod -aG i2c $USER  (logout/login lại)
    #   hoặc tạm thời: sudo chmod 666 /dev/i2c-8
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        parameters=[bno055_config],
        remappings=[
            ('/imu/imu', '/imu/data'),   # ← THÊM DÒNG NÀY
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 5 — IMU Reader (helper, optional)                          [t = 3.0s]
    # ══════════════════════════════════════════════════════════════════════════
    # Subscribe /imu/data, chuyển Quaternion → Euler, publish /imu/euler (rad).
    # Delay 3.0s để BNO055 driver khởi động và bắt đầu publish dữ liệu.
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
            LogInfo(msg='[view_robot] [3.0s] Starting imu_reader...'),
            imu_reader_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 6 — Extended Kalman Filter (robot_localization)            [t = 3.0s]
    # ══════════════════════════════════════════════════════════════════════════
    # Fuse /odom (wheel odometry) và /imu/data (BNO055) để:
    #   - Publish TF: odom → base_footprint  (thay thế cho static TF)
    #   - Publish /odometry/filtered         (odometry đã được fuse)
    #
    # Delay 3.0s để đảm bảo:
    #   - wheel_odom_node đã kết nối STM32 và publish /odom
    #   - bno055 đã khởi động và publish /imu/data
    #
    # Cấu hình trong ekf.yaml:
    #   odom0:  /odom       → fuse vx, yaw_rate
    #   imu0:   /imu/data   → fuse roll, pitch, yaw, angular_velocity
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )

    delayed_ekf = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg='[view_robot] [3.0s] Starting ekf_filter_node...'),
            ekf_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 7 — RPLIDAR S2E Driver (UDP)                                [t = 0s]
    # ══════════════════════════════════════════════════════════════════════════
    # Publish /scan (sensor_msgs/LaserScan, frame_id: laser_frame, ~10 Hz).
    # RPLIDAR S2E kết nối qua LAN cable với IP mặc định 192.168.11.2.
    #
    # scan_mode "Sensitivity": sample rate cao hơn, phạm vi 16m, tốt cho indoor
    # scan_mode "Standard":    phạm vi tối đa 30m nhưng sample rate thấp hơn
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type':     'udp',
            'udp_ip':           '192.168.11.2',  # IP của RPLIDAR S2E
            'udp_port':         8089,
            'frame_id':         'laser_frame',
            'inverted':         False,
            'angle_compensate': True,
            'scan_mode':        'Sensitivity',   # Tối ưu cho indoor mapping
            'use_sim_time':     use_sim_time,
        }]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 8 — Laser Scan Filter                                      [t = 10s]
    # ══════════════════════════════════════════════════════════════════════════
    # Lọc /scan → /scan_filtered (được SLAM Toolbox subscribe).
    # Áp dụng 2 bộ lọc theo thứ tự (laser_filter.yaml):
    #   filter1 (box_filter):   Loại bỏ điểm bên trong bounding box robot
    #                           để tránh self-detection (thân robot, dây cáp...)
    #   filter2 (range_filter): Loại bỏ điểm < 0.06m (nhiễu gần) và > 30m
    #
    # Delay 10.0s để chờ TF chain đầy đủ và ổn định:
    #   odom → base_footprint → base_link → chassis → laser_frame
    # Nếu vẫn còn warning "frame does not exist", tăng delay lên 12-15s.
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
            LogInfo(msg='[view_robot] [10.0s] Starting scan_to_scan_filter_chain...'),
            scan_filter_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # Launch Description — thứ tự khởi động
    # ══════════════════════════════════════════════════════════════════════════
    return LaunchDescription([
        declare_use_sim_time,

        # ── t = 0.0s ────────────────────────────────────────────────────────
        LogInfo(msg='[view_robot] ═══ Starting Full Hardware Bringup + EKF ═══'),
        LogInfo(msg='[view_robot] [0.0s] Starting RSP, wheel_odom, bno055, lidar...'),
        robot_state_publisher,
        wheel_odom_node,
        bno055_node,
        lidar_node,

        # ── t = 1.5s ────────────────────────────────────────────────────────
        delayed_jsp,

        # ── t = 3.0s ────────────────────────────────────────────────────────
        delayed_ekf,
        delayed_imu_reader,

        # ── t = 10.0s ───────────────────────────────────────────────────────
        delayed_scan_filter,
    ])