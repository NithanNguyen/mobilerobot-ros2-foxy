#!/usr/bin/env python3
# launch/auto_exploration_launch.py
# ──────────────────────────────────────────────────────────────────────────────
# Run on JETSON AGX Xavier — Hardware Bringup + EKF + SLAM Toolbox (mapping) +
# Nav2 (no AMCL/map_server) + Frontier Explorer (autonomous exploration).
#
# This file = hardware bringup (copied from nav_v2.launch.py) + exploration
#
# Stack:
#   1. robot_state_publisher      — URDF + static TF                   [t = 0.0s]
#   2. wheel_odom_node            — STM32 UART → /odom                 [t = 0.0s]
#   3. bno055                     — I2C → /imu/data (~100 Hz)          [t = 0.0s]
#   4. sllidar_node               — RPLIDAR S2E UDP → /scan            [t = 0.0s]
#   5. jetson_sensor_bridge       — STM32/Arduino UART → /ultrasonic/* [t = 0.0s]
#   6. joint_state_publisher      — /joint_states                      [t = 1.5s]
#   7. imu_reader                 — Quaternion → /imu/euler            [t = 3.0s]
#   8. ultrasonic_fusion_node     — /ultrasonic/* → /ultrasonic_scan   [t = 3.5s]
#   9. ekf_filter_node            — /odom + /imu/data → TF odom→base   [t = 7.0s]
#  10. scan_to_scan_filter_chain  — /scan → /scan_filtered             [t = 10.0s]
#  11. async_slam_toolbox_node    — Online Async SLAM (mapping)        [t = 12.0s]
#  12. nav2_bringup (navigation)  — Planner + Controller + BT          [t = 15.0s]
#  13. frontier_explorer          — Autonomous frontier exploration     [t = 20.0s]
#
# On shutdown (Ctrl+C / kill), if save_map_on_exit:=true the live /map is saved
# to map_save_path via nav2_map_server map_saver_cli.
#
# Environment (Jetson + Laptop must be the same):
#   export ROS_DOMAIN_ID=42
#   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#
# Run:
#   ros2 launch mobile_robot auto_exploration_launch.py
#   ros2 service call /explore/start std_srvs/srv/Trigger
# ──────────────────────────────────────────────────────────────────────────────

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    RegisterEventHandler,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # ── Package & shared paths ─────────────────────────────────────────────────
    package_name     = 'mobile_robot'
    pkg_share        = get_package_share_directory(package_name)
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    urdf_file         = os.path.join(pkg_share, 'urdf',   'mobile_robot.urdf.xacro')
    ekf_config        = os.path.join(pkg_share, 'config', 'ekf.yaml')
    bno055_config     = os.path.join(pkg_share, 'config', 'bno055_params.yaml')
    wheel_odom_config = os.path.join(pkg_share, 'config', 'wheel_odom_params.yaml')
    filter_config     = os.path.join(pkg_share, 'config', 'laser_filter.yaml')
    slam_config       = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    nav2_params       = os.path.join(pkg_share, 'config', 'nav2_exploration_params.yaml')

    # ── Launch arguments ───────────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Hardware always uses the wall clock. Keep false on physical hardware.'
    )

    declare_save_map_on_exit = DeclareLaunchArgument(
        'save_map_on_exit',
        default_value='true',
        description='Auto-save the live map when the launch is terminated (Ctrl+C / kill).'
    )

    declare_map_save_path = DeclareLaunchArgument(
        'map_save_path',
        default_value='/home/nguyenan/nvme_data/mbrobot_ws/src/mobile_robot/maps/demo/explored_map',
        description='Path (no extension) where the explored map is saved on exit.'
    )

    # Ultrasonic bridge serial config (required by jetson_sensor_bridge node).
    declare_us_serial_port = DeclareLaunchArgument(
        'us_serial_port',
        default_value='/dev/ttyUltrasonic',
        description='Serial port for ultrasonic sensor bridge (STM32/Arduino).'
    )

    declare_us_baud_rate = DeclareLaunchArgument(
        'us_baud_rate',
        default_value='115200',
        description='Baud rate for ultrasonic sensor serial port.'
    )

    use_sim_time     = LaunchConfiguration('use_sim_time')
    us_serial_port   = LaunchConfiguration('us_serial_port')
    us_baud_rate     = LaunchConfiguration('us_baud_rate')

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
            'publish_frequency': 50.0,
        }]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 3 — Wheel Odometry Node (STM32 via UART)                    [t = 0s]
    # ══════════════════════════════════════════════════════════════════════════
    # publish_tf = false (wheel_odom_params.yaml) — EKF handles TF odom→base_footprint
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
    # Do NOT add inline params — avoids overriding yaml and losing ros_topic_prefix.
    # remapping: /imu/imu → /imu/data so EKF receives the correct topic.
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
    # NODE 9 — RPLIDAR S2E Driver (UDP)                                [t = 0s]
    # ══════════════════════════════════════════════════════════════════════════
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
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
    # NODE 5 — Jetson Sensor Bridge (Ultrasonic UART reader)           [t = 0s]
    # ══════════════════════════════════════════════════════════════════════════
    # Provides /ultrasonic/* consumed by ultrasonic_fusion_node (t = 3.5s).
    # respawn=True: auto-restart on USB disconnect.
    jetson_sensor_bridge_node = Node(
        package='mobile_robot',
        executable='jetson_sensor_bridge.py',
        name='jetson_sensor_bridge',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            'use_sim_time': use_sim_time,
            'serial_port':  us_serial_port,
            'baud_rate':    us_baud_rate,
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
            LogInfo(msg='[explore] [1.5s] Starting joint_state_publisher...'),
            joint_state_publisher,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 6 — IMU Reader (Quaternion → Euler)                        [t = 3.0s]
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
            LogInfo(msg='[explore] [3.0s] Starting imu_reader...'),
            imu_reader_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 7 — Ultrasonic Fusion Node                                 [t = 3.5s]
    # ══════════════════════════════════════════════════════════════════════════
    ultrasonic_fusion_node = Node(
        package='mobile_robot',
        executable='ultrasonic_fusion_node.py',
        name='ultrasonic_fusion_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    delayed_ultrasonic_fusion = TimerAction(
        period=3.5,
        actions=[
            LogInfo(msg='[explore] [3.5s] Starting ultrasonic_fusion_node...'),
            ultrasonic_fusion_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 8 — EKF (robot_localization)                               [t = 7.0s]
    # ══════════════════════════════════════════════════════════════════════════
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
            LogInfo(msg='[explore] [7.0s] Starting ekf_filter_node...'),
            ekf_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 10 — Laser Scan Filter                                     [t = 10s]
    # ══════════════════════════════════════════════════════════════════════════
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
            LogInfo(msg='[explore] [10.0s] Starting scan_to_scan_filter_chain...'),
            scan_filter_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 11 — SLAM Toolbox (Online Async, mapping)                  [t = 12s]
    # ══════════════════════════════════════════════════════════════════════════
    # Publishes /map directly and broadcasts TF map→odom (replaces AMCL + map_server).
    slam_toolbox = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[explore] [12.0s] Starting async_slam_toolbox_node (mapping)...'),
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_config,
                    {'use_sim_time': False, 'scan_topic': '/scan_filtered'}
                ]
            ),
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 12 — Nav2 Bringup (navigation only, no AMCL/map_server)    [t = 15s]
    # ══════════════════════════════════════════════════════════════════════════
    nav2_bringup = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='[explore] [15.0s] Starting Nav2 navigation bringup...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file':  nav2_params,
                }.items()
            ),
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 13 — Frontier Explorer                                      [t = 20s]
    # ══════════════════════════════════════════════════════════════════════════
    frontier_explorer = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='[explore] [20.0s] Starting frontier_explorer...'),
            Node(
                package='mobile_robot',
                executable='frontier_explorer.py',
                name='frontier_explorer',
                output='screen',
                parameters=[{
                    'use_sim_time':              False,
                    'min_frontier_size':         8,
                    'frontier_min_distance':     0.80,
                    'goal_tolerance':            0.35,
                    'exploration_timeout':       1200.0,
                    'spin_on_arrival':           False,
                    'w_dist':                    3.0,
                    'w_rot':                     2.5,
                    'w_fov':                     2.0,
                    'w_clearance':               1.5,
                    'fov_penalty_side':          1.5,
                    'fov_penalty_edge':          3.0,
                    'clearance_search_radius':   0.80,
                    'rear_scan_trigger_distance': 2.5,
                    'rear_scan_yaw':             1.8,
                    'frontier_retry_limit':      2,
                }]
            ),
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # Map save on shutdown (conditional on save_map_on_exit)
    # ══════════════════════════════════════════════════════════════════════════
    save_map_cmd = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('save_map_on_exit')),
        cmd=[
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', LaunchConfiguration('map_save_path'),
            '--ros-args', '-p', 'map_subscribe_transient_local:=true'
        ],
        output='screen'
    )
    save_map_handler = RegisterEventHandler(
        OnShutdown(on_shutdown=[save_map_cmd])
    )

    # ══════════════════════════════════════════════════════════════════════════
    # Launch Description
    # ══════════════════════════════════════════════════════════════════════════
    return LaunchDescription([
        # 1. Launch arguments
        declare_use_sim_time,
        declare_save_map_on_exit,
        declare_map_save_path,
        declare_us_serial_port,
        declare_us_baud_rate,

        LogInfo(msg='[explore] ═══ Starting Hardware Bringup + SLAM + Nav2 + Frontier Explorer ═══'),
        LogInfo(msg='[explore] [0.0s] Starting RSP, wheel_odom, bno055, lidar, sensor_bridge...'),

        # 2. t = 0.0s hardware nodes
        robot_state_publisher,
        wheel_odom_node,
        bno055_node,
        lidar_node,
        jetson_sensor_bridge_node,
        delayed_jsp,
        delayed_imu_reader,
        delayed_ultrasonic_fusion,
        delayed_ekf,
        delayed_scan_filter,
        slam_toolbox,
        nav2_bringup,
        frontier_explorer,
        save_map_handler,
    ])
