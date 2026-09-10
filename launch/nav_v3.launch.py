#!/usr/bin/env python3
# launch/nav_v3.launch.py
# ──────────────────────────────────────────────────────────────────────────────
# Run on JETSON AGX Xavier — Full Hardware Bringup + EKF + Nav2
#
# Stack:
#   1. robot_state_publisher      — URDF + static TF                   [t = 0.0s]
#   2. joint_state_publisher      — /joint_states                      [t = 1.5s]
#   3. wheel_odom_node            — STM32 UART → /odom                 [t = 0.0s]
#   4. bno055                     — I2C → /imu/data (~100 Hz)          [t = 0.0s]
#   5. jetson_sensor_bridge       — STM32/Arduino UART → /ultrasonic/* [t = 0.0s]
#   6. imu_reader                 — Quaternion → /imu/euler            [t = 3.0s]
#   7. ultrasonic_fusion_node     — /ultrasonic/* → /ultrasonic_scan   [t = 3.5s]
#   8. ekf_filter_node            — /odom + /imu/data → TF odom→base   [t = 7.0s]
#   9. sllidar_node               — RPLIDAR S2E UDP → /scan            [t = 0.0s]
#  10. scan_to_scan_filter_chain  — /scan → /scan_filtered             [t = 10.0s]
#  11. nav2_bringup               — AMCL + Planner + Controller + BT   [t = 12.0s]
#  12. global_localizer           — Autonomous startup localization    [t = 14.0s]
#  13. checkpoint_navigator       — Autonomous waypoint sequencer      [t = 20.0s]
#
# Environment (Jetson + Laptop must be the same):
#   export ROS_DOMAIN_ID=42
#   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#
# Run local with default area (floor e6):
#   ros2 launch mobile_robot nav_v2_launch.py
#
# Run for a specific floor (e.g., e1):
#   ros2 launch mobile_robot nav_v2_launch.py floor:=e1
#
# Run with experiment metadata:
#   ros2 launch mobile_robot nav_v3.launch.py scenario:=S1 run_number:=01
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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import datetime
from launch.actions import ExecuteProcess, OpaqueFunction


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
    nav2_params       = os.path.join(pkg_share, 'config', 'nav2_params_test.yaml')

    # ── Launch arguments ───────────────────────────────────────────────────────
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true. Must be false on physical hardware.'
    )

    declare_floor = DeclareLaunchArgument(
        'floor',
        default_value='a1',
        description='Floor identifier (e.g., e6, e1)'
    )

    declare_timeout = DeclareLaunchArgument(
        'timeout_at_checkpoint',
        default_value='30.0',
        description='Dwell time in seconds at each checkpoint before returning to home.'
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
        default_value='S1',
        description='Experiment scenario ID (e.g. S1, S3, S6). Used in bag filename.'
    )

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
    floor               = LaunchConfiguration('floor')
    timeout             = LaunchConfiguration('timeout_at_checkpoint')
    us_serial_port      = LaunchConfiguration('us_serial_port')
    us_baud_rate        = LaunchConfiguration('us_baud_rate')
    scenario    = LaunchConfiguration('scenario')
    run_number  = LaunchConfiguration('run_number')
    record_bag  = LaunchConfiguration('record_bag')

    dynamic_map_file = [
        pkg_share, '/maps/map_', floor, '.yaml'
    ]

    dynamic_checkpoint_file = [
        pkg_share, '/config/checkpoints_', floor, '.yaml'
    ]

    # ══════════════════════════════════════════════════════════════════════════
    # BAG RECORDING — ros2 bag record (OpaqueFunction for dynamic path)
    # ══════════════════════════════════════════════════════════════════════════
    def _make_bag_record_action(context, *args, **kwargs):
        """
        Builds the ros2 bag record ExecuteProcess at launch-time so the
        timestamp is captured when the launch actually runs, not when the
        LaunchDescription is constructed.

        Bag path: ~/mbrobot_ws/bags/s{scenario}_run{run_number}_{YYYYMMDD_HHMMSS}
        Recording is skipped when record_bag != 'true'.
        """
        do_record = context.launch_configurations.get('record_bag', 'true').lower() == 'true'
        if not do_record:
            return [LogInfo(msg='[nav] [bag] record_bag=false — bag recording skipped.')]

        sc  = context.launch_configurations.get('scenario',   'S1')
        rn  = context.launch_configurations.get('run_number', '01')
        ts  = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_name = f's{sc}_run{rn}_{ts}'
        ws = os.environ.get('WS', os.path.join(os.path.expanduser('~'), 'mbrobot_ws'))
        bags_dir = os.path.join(ws, 'bags')
        bag_path = os.path.join(bags_dir, bag_name)

        os.makedirs(bags_dir, exist_ok=True)

        topics = [
            '/scan',
            '/scan_filtered',
            '/odom',
            '/odometry/filtered',
            '/imu/data',
            '/amcl_pose',
            '/cmd_vel',
            '/tf',
            '/tf_static',
            '/ultrasonic_scan',
            '/robot/state',
            '/robot/status_message',
            '/plan',                                    # P2: mẫu số PLR (bag_plan_length.py)
            '/navigate_to_pose/_action/feedback',       # P2: RC = number_of_recoveries (bag_to_rc.py)
            '/ultrasonic/us_top_left',
            '/ultrasonic/us_top_right',
            '/ultrasonic/us_mid_left',
            '/ultrasonic/us_mid_right',
            '/ultrasonic/us_bot_left',
            '/ultrasonic/us_bot_right',
        ]

        return [
            LogInfo(msg=f'[nav] [bag] Recording → {bag_path}'),
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-o', bag_path, *topics],
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
    # NODE 5 — Jetson Sensor Bridge (Ultrasonic UART reader)           [t = 0s]
    # ══════════════════════════════════════════════════════════════════════════
    # Đọc frame "$v0,v1,...,v7*chk" từ STM32/Arduino qua UART
    # và publish từng sensor thành /ultrasonic/<name> (sensor_msgs/Range).
    # Topic map: index 0..5 → us_top_left, us_top_right, us_mid_left,
    #                          us_mid_right, us_bot_left, us_bot_right
    #
    # respawn=True: tự restart nếu mất kết nối USB
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
            LogInfo(msg='[nav] [3.0s] Starting imu_reader...'),
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
            LogInfo(msg='[nav] [3.5s] Starting ultrasonic_fusion_node...'),
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
            LogInfo(msg='[nav] [7.0s] Starting ekf_filter_node...'),
            ekf_node,
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
        period=11.0,
        actions=[
            LogInfo(msg='[nav] [10.0s] Starting scan_to_scan_filter_chain...'),
            scan_filter_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 11 — Nav2 Bringup                                          [t = 12s]
    # ══════════════════════════════════════════════════════════════════════════
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map':          dynamic_map_file,
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params,
        }.items()
    )

    delayed_nav2 = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[nav] [12.0s] Starting Nav2 bringup...'),
            nav2_bringup,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 12 — Global Localizer                                       [t = 14s]
    # ══════════════════════════════════════════════════════════════════════════
    # Autonomous startup localization — replaces the old hardcoded set_initial_pose
    # ExecuteProcess. Waits for AMCL, calls /reinitialize_global_localization, spins 360° to gather LiDAR observations, then confirms the converged pose and publishes /global_localizer/ready.
    # Threshold values can be tuned from nav2_params_test.yaml without recompiling.
    global_localizer_node = Node(
        package='mobile_robot',
        executable='global_localizer.py',
        name='global_localizer',
        output='screen',
        parameters=[{
            'use_sim_time':              use_sim_time,
            'convergence_threshold_xy':  0.10,
            'convergence_threshold_yaw': 0.15,
            'spin_angular_velocity':     0.4,
            'max_spin_attempts':         3,
            'amcl_wait_timeout':         30.0,
        }]
    )

    delayed_global_localizer = TimerAction(
        period=14.0,
        actions=[
            LogInfo(msg='[nav] [14.0s] Starting global_localizer (autonomous localization)...'),
            global_localizer_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 12 - SET INITIAL POSE                                       [t = 15s]
    # ══════════════════════════════════════════════════════════════════════════
    set_initial_pose = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/initialpose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            # e6 - Home
            '{"header": {"frame_id": "map"}, "pose": {"pose": {"position": {"x": 6.6810, "y": -5.1279, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.999671, "w": 0.025644}}, "covariance": [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.068]}}'
            # a1 - Sanh chinh
            # '{"header": {"frame_id": "map"}, "pose": {"pose": {"position": {"x": -45.612, "y": 7.584, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": -0.918, "w": 0.396}}, "covariance": [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.068]}}'
        ],
        output='screen'
    )
    # Setting estimate pose: Frame:map, Position(-45.6767, 7.93121, 0), Orientation(0, 0, -0.911428, 0.411461) = Angle: -2.29348

    delayed_initial_pose = TimerAction(
        period=15.0,
        actions=[
            LogInfo(msg='[nav] [15.0s] Publishing initial pose to AMCL...'),
            set_initial_pose,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # NODE 13 — Checkpoint Navigator                                   [t = 20s]
    # ══════════════════════════════════════════════════════════════════════════
    checkpoint_navigator_node = Node(
        package='mobile_robot',
        executable='navigator.py',
        name='navigator',
        output='screen',
        parameters=[{
            'use_sim_time':          use_sim_time,
            'checkpoint_file':       dynamic_checkpoint_file,
            'timeout_at_checkpoint': timeout,
            'home_checkpoint_id':    0,
        }]
    )

    delayed_checkpoint_nav = TimerAction(
        period=20.0,
        actions=[
            LogInfo(msg='[nav] [20.0s] Starting checkpoint_navigator...'),
            checkpoint_navigator_node,
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    # Launch Description
    # ══════════════════════════════════════════════════════════════════════════
    return LaunchDescription([
        declare_use_sim_time,
        declare_floor,
        declare_timeout,
        declare_us_serial_port,
        declare_us_baud_rate,
        declare_scenario,
        declare_run_number,
        declare_record_bag,

        LogInfo(msg='[nav] ═══ Starting Full Hardware Bringup + EKF + Nav2 ═══'),
        LogInfo(msg='[nav] [0.0s] Starting RSP, wheel_odom, bno055, lidar, sensor_bridge...'),

        bag_record_action,
        robot_state_publisher,
        wheel_odom_node,
        bno055_node,
        lidar_node,
        # jetson_sensor_bridge_node,
        # delayed_ultrasonic_fusion,
        delayed_jsp,
        delayed_imu_reader,
        delayed_ekf,
        delayed_scan_filter,
        delayed_nav2,
        # delayed_global_localizer,
        delayed_initial_pose,
        delayed_checkpoint_nav,
    ])
