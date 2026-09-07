# Receptionist Robot
## Details

Jetson AGX Xavier Developer Kit 16GB (NVIDIA Xavier SoC) 

Board: X221 https://auvidea.eu/product/x221-70415/

X221_Manual https://auvidea.eu/download/X221_Manual_v2.0.pdf

OS: Ubuntu 20.04

ROS 2 Foxy

## Build and Source workspace

```bash
cd ~/mbrobot_ws
colcon build --packages-select mobile_robot
source install/setup.bash
```

## CHECK

```bash
chmod +x ~/mbrobot_ws/pre_check.sh
cd ~/mbrobot_ws
# Kiểm tra đầy đủ, tầng e6 (mặc định) — KHÔNG launch node ROS
./pre_check.sh

# Kiểm tra tầng e1
./pre_check.sh e1
# Kiểm tra tầng e1, bỏ qua bước build (đã build rồi)
./pre_check.sh e1 --skip-build
# Kiểm tra đầy đủ, baogồm launch node + đo Hz topic + kiểm tra TF
./pre_check.sh e6 --check-topics
# Kiểm tra tầng e1 với đầy đủ topic/TF
./pre_check.sh e1 --skip-build --check-topics
# Xem hướng dẫ
./pre_check.sh --help
```

## QUICK RUN

```bash
cd ~/mbrobot_ws
./run_nav.sh
```

## RUN STEP BY STEP

```bash
# Jetson
Terminal 1:
ros2 launch mobile_robot slam.launch.py
ros2 launch mobile_robot nav.launch.py
Terminal 2:
ros2 run mobile_robot checkpoint_cmd.py

# Laptop
ros2 launch mobile_robot rviz_launch.py

# Lưu map
ros2 topic hz /map
# Cách 1: Tăng Timeout và cấu hình QoS (Khuyên dùng)
ros2 run nav2_map_server map_saver_cli \
  -f ~/mbrobot_ws/src/mobile_robot/maps/map_e6 \
  --ros-args \
    -p save_map_timeout:=60000 \
    -p map_subscribe_transient_local:=true \
    -p free_thresh_default:=0.25 \
    -p occupied_thresh_default:=0.65
# Cách 2: Sử dụng Service GetMap thay vì Topic
ros2 run nav2_map_server map_saver_cli -f lab_map --ros-args -p map_subscribe_transient_local:=true -p free_thresh_default:=0.25 -p occupied_thresh_default:=0.65 --ros-args -p save_map_timeout:=30000 --remap map:=/map
# Cách 3:
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name: {data: '/home/nguyenan/lab_map'}"

```

# Debug

Init LiDAR

```bash
sudo ip addr add 192.168.11.1/24 dev enp2s0
sudo ip link set enp2s0 up
```

Ultrasonic

```bash
python3 /home/nguyenan/mbrobot_ws/src/mobile_robot/scripts/jetson_sensor_bridge.py --debug
```

IMU

```bash
# Terminal 1
ros2 run bno055 bno055 \
  --ros-args \
  --params-file ~/mbrobot_ws/src/bno055/bno055/params/bno055_params.yaml \
  --remap /imu/imu:=/imu/data
# Terminal 2 - Kiểm tra topic
ros2 topic list | grep imu
# /imu/data
# /imu/mag
# /imu/temp
# /imu/calib_status

# Xem dữ liệu raw
ros2 topic echo /imu/data
# Kiểm tra tần số
ros2 topic hz /imu/data
```

Wheel odom

```bash
# terminal 1
ros2 run mobile_robot wheel_odom_node --ros-args --params-file src/mobile_robot/config/wheel_odom_params.yaml
# terminal 2
source ~/mbrobot_ws/install/setup.bash
ros2 topic list
ros2 topic hz /odom
ros2 topic echo /odom
```

EKF

```bash
ros2 run robot_localization ekf_node --ros-args --params-file src/mobile_robot/config/ekf.yaml
ros2 topic echo /odometry/filtered
```

Check Nodes run

```bash
ros2 node list
# Lọc bớt các node hệ thống và chỉ xem các node quan trọng
ros2 node list | grep server
# Xem node nào kết nối với node nào
rqt_graph
```

## Finite State Machine

