# Troubleshooting

Per-subsystem bring-up checks. Run each one in isolation before blaming the full
stack — the launch files start ten processes and a single silent device makes the
failure look like a Nav2 problem.

Commands assume the workspace is at `~/mbrobot_ws` and has been sourced:

```bash
source ~/mbrobot_ws/install/setup.bash
```

## Pre-flight check

`pre_check.sh` runs six phases: build, system prerequisites, hardware and devices,
ROS topics and TF, configuration files for the selected floor, and disk / memory /
temperature.

```bash
./pre_check.sh --help
./pre_check.sh --floor e6                       # full check, floor e6
./pre_check.sh --floor e1 --skip-build          # skip the colcon build phase
./pre_check.sh --floor e6 --check-topics        # also launch nodes and measure topic rates
```

Phase 0 rebuilds the workspace and phase 3 with `--check-topics` launches nodes in
the background, so this is not a read-only script.

## LiDAR link

The RPLIDAR S2E is reached over Ethernet/UDP. The Jetson interface needs a static
address on the sensor's subnet before `sllidar_node` can connect. Replace the
interface name with whichever one is wired to the sensor:

```bash
sudo ip addr add 192.168.11.1/24 dev enp2s0
sudo ip link set enp2s0 up
```

Then check the scan is flowing:

```bash
ros2 topic hz /scan
ros2 topic hz /scan_filtered
```

If `/scan` is alive but `/scan_filtered` is not, the filter chain is failing its TF
lookup to `base_footprint` rather than the LiDAR being at fault.

## Ultrasonic bridge

Run the bridge directly to see the raw frames arriving from the ESP32:

```bash
python3 src/mobile_robot/scripts/jetson_sensor_bridge.py --debug
```

A frame is accepted only if it starts with `$`, contains `*`, carries exactly six
fields, and matches its XOR checksum. Silent output with no errors means the port
opened but nothing is being received — check `/dev/ttyUltrasonic` and the ESP32.

```bash
ros2 topic echo /ultrasonic/us_top_left
ros2 topic hz /ultrasonic_scan
```

## IMU

```bash
ros2 run bno055 bno055 \
  --ros-args \
  --params-file src/mobile_robot/config/bno055_params.yaml \
  --remap /imu/imu:=/imu/data
```

In a second terminal:

```bash
ros2 topic list | grep imu     # expect /imu/data /imu/mag /imu/temp /imu/calib_status
ros2 topic echo /imu/data
ros2 topic hz /imu/data
```

The remap matters: without it the driver publishes on `/imu/imu` and the EKF
subscribes to nothing.

## Wheel odometry

```bash
ros2 run mobile_robot wheel_odom_node \
  --ros-args --params-file src/mobile_robot/config/wheel_odom_params.yaml
```

```bash
ros2 topic hz /odom
ros2 topic echo /odom
```

The node opens `/dev/ttyWheel` once at startup and does not reconnect, so a serial
error leaves the node alive but permanently silent. Restart it rather than waiting.

## EKF

```bash
ros2 run robot_localization ekf_node \
  --ros-args --params-file src/mobile_robot/config/ekf.yaml
ros2 topic echo /odometry/filtered
```

The EKF is the only publisher of `TF odom -> base_footprint`. If that transform is
missing, check that `wheel_odom_node` has `publish_tf: false` and that both `/odom`
and `/imu/data` are live — the filter will not publish until it has an input.

## Inspecting the running graph

```bash
ros2 node list
ros2 node list | grep server      # Nav2 lifecycle servers only
rqt_graph
ros2 param get /bt_navigator default_bt_xml_filename
ros2 run tf2_tools view_frames
```

## Common ordering symptom

`canTransform` warnings in the first seconds of a run are expected: Nav2 activates
at t=12 s while `global_localizer` only starts at t=14 s, so there is a short window
with no `map -> odom` transform. Warnings that persist past roughly t=20 s are a
real localization failure, not a start-up artefact.
