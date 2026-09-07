# launch/view_lidar_imu_launch.py
# ──────────────────────────────────────────────────────────────────────────────
# Chạy trên JETSON AGX Xavier qua SSH từ laptop
#
# Mục đích: Xem đồng thời dữ liệu LiDAR (RPLIDAR S2E) và IMU (BNO055)
#           mà KHÔNG cần EKF, SLAM hay Navigation.
#
# Bao gồm:
#   1. robot_state_publisher        — publish /robot_description + TF tĩnh từ URDF
#   2. joint_state_publisher        — publish /joint_states (bánh xe)
#   3. static_transform_publisher   — odom → base_footprint (thay EKF)
#   4. sllidar_node                 — driver RPLIDAR S2E qua UDP
#   5. scan_to_scan_filter_chain    — lọc /scan → /scan_filtered
#   6. bno055 node                  — driver IMU BNO055 qua I2C (J23 / i2c-8)
#   7. imu_reader node              — chuyển Quaternion → Euler, publish /imu/euler
#
# Topics chính:
#   /scan                   raw LaserScan  (RPLIDAR S2E, frame: laser_frame)
#   /scan_filtered          filtered LaserScan
#   /imu/data               sensor_msgs/Imu  (BNO055, frame: imu_link)
#   /imu/euler              geometry_msgs/Vector3  (roll/pitch/yaw, rad)
#   /imu/temperature        sensor_msgs/Temperature
#   /robot_description      URDF string
#   /tf, /tf_static         TF tree
#
# Thứ tự khởi động:
#   0.0s  robot_state_publisher, static_tf_odom, lidar_node, bno055_node
#   1.5s  joint_state_publisher   (chờ robot_description publish trước)
#   3.0s  imu_reader              (chờ BNO055 driver khởi động xong)
#   8.0s  scan_to_scan_filter_chain (chờ TF chain hoàn chỉnh)
#
# Yêu cầu môi trường (cả Jetson lẫn Laptop):
#   export ROS_DOMAIN_ID=42
#   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#
# Chạy trên Jetson (qua SSH):
#   ros2 launch mobile_robot view_lidar_imu_launch.py
#
# Chạy RViz trên Laptop (sau khi Jetson đã up):
#   ros2 launch mobile_robot rviz_launch.py
#
# Kiểm tra nhanh IMU trên terminal:
#   ros2 topic echo /imu/data
#   ros2 topic hz  /imu/data          # kỳ vọng ~100 Hz
#   ros2 topic echo /imu/euler
# ──────────────────────────────────────────────────────────────────────────────

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'mobile_robot'
    pkg_share    = get_package_share_directory(package_name)

    # ── Paths ──────────────────────────────────────────────────────────────────
    urdf_model    = os.path.join(pkg_share, 'urdf',   'mobile_robot.urdf.xacro')
    filter_config = os.path.join(pkg_share, 'config', 'laser_filter.yaml')
    bno055_config = os.path.join(pkg_share, 'config', 'bno055_params.yaml')

    use_sim_time = False

    # ── 1. Robot State Publisher ───────────────────────────────────────────────
    # Publish /robot_description và các TF cố định từ URDF:
    #   base_footprint → base_link → chassis → laser_frame
    #                                        → imu_link
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time':      use_sim_time,
            'robot_description': Command(['xacro ', urdf_model]),
            'publish_frequency': 10.0,   # Hz — đủ để laptop nhận qua WiFi/LAN
        }]
    )

    # ── 2. Joint State Publisher (delay 1.5 s) ─────────────────────────────────
    # Chờ /robot_description sẵn sàng trước khi publish /joint_states.
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
            LogInfo(msg='[view_lidar_imu] Starting joint_state_publisher...'),
            joint_state_publisher,
        ]
    )

    # ── 3. Static TF: odom → base_footprint ───────────────────────────────────
    # Thay thế EKF cho launch xem cảm biến đơn giản.
    # Robot được gắn tạm tại gốc odom — đủ để RViz hiển thị scan & IMU.
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_base',
        output='screen',
        arguments=[
            '0', '0', '0',       # x  y  z
            '0', '0', '0', '1',  # qx qy qz qw  (không xoay)
            'odom', 'base_footprint'
        ]
    )

    # ── 4. RPLIDAR S2E Driver (UDP) ────────────────────────────────────────────
    # Publish /scan (sensor_msgs/LaserScan, frame_id: laser_frame)
    # RPLIDAR S2E kết nối qua LAN cable, IP mặc định: 192.168.11.2
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type':     'udp',
            'udp_ip':           '192.168.11.2',   # IP của RPLIDAR S2E
            'udp_port':         8089,
            'frame_id':         'laser_frame',
            'inverted':         False,
            'angle_compensate': True,
            'scan_mode':        'Sensitivity',
            'use_sim_time':     use_sim_time,
        }]
    )

    # ── 5. Laser Filter (delay 8.0 s) ─────────────────────────────────────────
    # Chờ toàn bộ TF chain ổn định:
    #   odom → base_footprint → base_link → chassis → laser_frame
    # Nếu vẫn thấy cảnh báo "frame does not exist", tăng thêm 2–3 s.
    scan_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[filter_config, {'use_sim_time': use_sim_time}]
    )

    delayed_scan_filter = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='[view_lidar_imu] Starting scan_to_scan_filter_chain...'),
            scan_filter_node,
        ]
    )

    # ── 6. BNO055 IMU Driver (I2C) ─────────────────────────────────────────────
    # Kết nối qua I2C bus 8 (/dev/i2c-8) trên cổng J23 của X221-AI.
    #
    # Topics được publish:
    #   /imu/data            sensor_msgs/Imu         (~100 Hz)
    #   /imu/mag             sensor_msgs/MagneticField
    #   /imu/temp            sensor_msgs/Temperature
    #   /imu/calib_status    std_msgs/UInt8
    #
    # Lưu ý: bno055 driver yêu cầu quyền truy cập /dev/i2c-8.
    #   Nếu gặp lỗi Permission denied:
    #     sudo usermod -aG i2c $USER  (sau đó logout/login lại)
    #   hoặc tạm thời:
    #     sudo chmod 666 /dev/i2c-8
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

    # ── 7. IMU Reader (delay 3.0 s) ────────────────────────────────────────────
    # Chờ BNO055 driver khởi động và bắt đầu publish /imu/data.
    # Chuyển đổi Quaternion → Euler và publish /imu/euler (Vector3, rad).
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
            LogInfo(msg='[view_lidar_imu] Starting imu_reader...'),
            imu_reader_node,
        ]
    )

    # ── Launch Description ─────────────────────────────────────────────────────
    return LaunchDescription([
        # ── 0.0 s ──────────────────────────────────────────────────────────────
        LogInfo(msg='[view_lidar_imu] === Starting LiDAR + IMU viewer ==='),
        LogInfo(msg='[view_lidar_imu] Starting robot_state_publisher, static_tf, lidar, bno055...'),
        robot_state_publisher,
        static_tf_odom,
        lidar_node,
        bno055_node,

        # ── 1.5 s ──────────────────────────────────────────────────────────────
        delayed_jsp,

        # ── 3.0 s ──────────────────────────────────────────────────────────────
        delayed_imu_reader,

        # ── 8.0 s ──────────────────────────────────────────────────────────────
        delayed_scan_filter,
    ])