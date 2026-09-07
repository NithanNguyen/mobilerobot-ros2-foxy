#!/usr/bin/env python3
# launch/slam_v3_launch.py
# ──────────────────────────────────────────────────────────────────────────────
# SLAM mapping bringup — hardware + slam_toolbox (NO Nav2 / no autonomy)
#
# Stack:
#   1. robot_state_publisher      — URDF + static TF                   [t = 0.0s]
#   2. joint_state_publisher      — /joint_states                      [t = 1.5s]
#   3. wheel_odom_node            — STM32 UART → /odom                 [t = 0.0s]
#   4. bno055                     — I2C → /imu/data                    [t = 0.0s]
#   5. sllidar_node               — RPLIDAR S2E UDP → /scan            [t = 0.0s]
#   6. imu_reader                 — Quaternion → /imu/euler            [t = 3.0s]
#   7. ekf_filter_node            — /odom + /imu/data → TF odom→base   [t = 7.0s]
#   8. scan_to_scan_filter_chain  — /scan → /scan_filtered             [t = 10.0s]
#   9. slam_toolbox (online_async)— /scan_filtered → /map + TF         [t = 12.0s]
#  10. bag_record                 — ros2 bag record                    [t = 0.0s]
#
# Drive manually with teleop_twist_keyboard or joystick after launch.
#
# Run:
#   ros2 launch mobile_robot slam_v3_launch.py
#   ros2 launch mobile_robot slam_v3_launch.py scenario:=S6 speed:=0.20 run_number:=02
# ──────────────────────────────────────────────────────────────────────────────

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import datetime
from launch.actions import ExecuteProcess, OpaqueFunction


def generate_launch_description():
    # ── Package & shared paths ─────────────────────────────────────────────────
    package_name     = 'mobile_robot'
    pkg_share        = get_package_share_directory(package_name)

    urdf_file         = os.path.join(pkg_share, 'urdf',   'mobile_robot.urdf.xacro')
    ekf_config        = os.path.join(pkg_share, 'config', 'ekf.yaml')
    bno055_config     = os.path.join(pkg_share, 'config', 'bno055_params.yaml')
    wheel_odom_config = os.path.join(pkg_share, 'config', 'wheel_odom_params.yaml')
    filter_config     = os.path.join(pkg_share, 'config', 'laser_filter.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

    # ── Launch arguments ───────────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true. Must be false on physical hardware.'
    )

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

    declare_scenario = DeclareLaunchArgument(
        'scenario',
        default_value='S6',
        description='Always S6 for SLAM runs.'
    )

    declare_speed = DeclareLaunchArgument(
        'speed',
        default_value='0.10',
        description=(
            'Teleop reference speed in m/s for this run (0.10 / 0.20 / 0.30). '
            'Metadata only — used in bag and map filenames. '
            'Actual speed is set by the operator via teleop.'
        )
    )
    speed = LaunchConfiguration('speed')

    declare_run_number = DeclareLaunchArgument(
        'run_number',
        default_value='01',
        description='Zero-padded run counter within the scenario (e.g. 01, 02).'
    )

    declare_record_bag = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='Set false to skip bag recording (e.g. dry-run or debug sessions).'
    )

    use_sim_time        = LaunchConfiguration('use_sim_time')
    us_serial_port      = LaunchConfiguration('us_serial_port')
    us_baud_rate        = LaunchConfiguration('us_baud_rate')
    scenario    = LaunchConfiguration('scenario')
    run_number  = LaunchConfiguration('run_number')
    record_bag  = LaunchConfiguration('record_bag')

    # ══════════════════════════════════════════════════════════════════════════
    # BAG RECORDING — ros2 bag record (OpaqueFunction for dynamic path)
    # ══════════════════════════════════════════════════════════════════════════
    def _make_bag_record_action(context, *args, **kwargs):
        """
        Bag path: ~/mbrobot_ws/bags/s{scenario}_v{speed_cm:03d}_run{run_number}_{ts}
        Recording skipped when record_bag != 'true'.
        """
        do_record = context.launch_configurations.get('record_bag', 'true').lower() == 'true'
        if not do_record:
            return [LogInfo(msg='[slam] [bag] record_bag=false — bag recording skipped.')]

        sc  = context.launch_configurations.get('scenario',   'S6')
        sp  = context.launch_configurations.get('speed',      '0.10')
        rn  = context.launch_configurations.get('run_number', '01')
        ts  = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

        speed_cm = int(float(sp) * 100)
        bag_name = f's{sc}_v{speed_cm:03d}_run{rn}_{ts}'
        bags_dir = os.path.join(os.path.expanduser('~'), 'mbrobot_ws', 'bags')
        bag_path = os.path.join(bags_dir, bag_name)

        os.makedirs(bags_dir, exist_ok=True)

        topics = [
            '/scan',
            '/scan_filtered',
            '/odom',
            '/odometry/filtered',
            '/imu/data',
            '/cmd_vel',
            '/tf',
            '/tf_static',
            '/map',
        ]

        return [
            LogInfo(msg=f'[slam] [bag] Recording → {bag_path}'),
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-o', bag_path] + topics,
                output='screen',
            ),
        ]

    bag_record_action = OpaqueFunction(function=_make_bag_record_action)

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
            LogInfo(msg='[nav] [1.5s] Starting joint_state_publisher...'),
            joint_state_publisher,
        ]
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
            LogInfo(msg='[nav] [3.0s] Starting imu_reader...'),
            imu_reader_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 6 — EKF (robot_localization)                               [t = 7.0s]
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
            LogInfo(msg='[nav] [7.0s] Starting ekf_filter_node...'),
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
    # NODE 8 — Laser Scan Filter                                     [t = 10s]
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
            LogInfo(msg='[nav] [10.0s] Starting scan_to_scan_filter_chain...'),
            scan_filter_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 9 — slam_toolbox (online_async mapping)                    [t = 12s]
    # ══════════════════════════════════════════════════════════════════════════
    # Scan source: /scan_filtered (after laser_filters chain)
    # Publishes: /map (nav_msgs/OccupancyGrid), TF map→odom
    # Config file: config/mapper_params_online_async.yaml (tuned for BNO055 + Jetson)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time},
        ]
    )

    delayed_slam = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[slam] [12.0s] Starting slam_toolbox (online_async)...'),
            slam_toolbox_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # Launch Description
    # ══════════════════════════════════════════════════════════════════════════
    return LaunchDescription([
        declare_use_sim_time,
        declare_us_serial_port,
        declare_us_baud_rate,
        declare_scenario,
        declare_speed,
        declare_run_number,
        declare_record_bag,

        LogInfo(msg='[slam] ═══ Starting SLAM Mapping Bringup ═══'),
        LogInfo(msg='[slam] [0.0s] Starting RSP, wheel_odom, bno055, lidar...'),

        bag_record_action,
        robot_state_publisher,
        wheel_odom_node,
        bno055_node,
        lidar_node,
        delayed_jsp,
        delayed_imu_reader,
        delayed_ekf,
        delayed_scan_filter,
        delayed_slam,
    ])
