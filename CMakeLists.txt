cmake_minimum_required(VERSION 3.5)
project(mobile_robot)

# ── C++ standards ──────────────────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Find packages ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ── Python node: imu_reader ────────────────────────────────────────────────────
# Cài imu_reader.py dưới dạng executable (ros2 run mobile_robot imu_reader)
ament_python_install_package(${PROJECT_NAME})

# ── Install IMU ────────────────────────────────────────────────────────
install(PROGRAMS scripts/imu_reader.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_reader)

# ── Install wheel_odom_node ────────────────────────────────────────────────────────
install(PROGRAMS scripts/wheel_odom_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wheel_odom_node)
  
install(
  PROGRAMS
    scripts/robot_teleop.py
    scripts/navigator.py
    scripts/global_localizer.py
    scripts/checkpoint_cmd.py
    scripts/frontier_explorer.py
    scripts/ultrasonic_fusion_node.py
    scripts/endurance_test.py
    scripts/tools/save_checkpoints.py
    scripts/tools/vel_compare_logger.py
    scripts/tools/monitor_nodes_nav2.py
  DESTINATION lib/${PROJECT_NAME}
)
# ── Install launch files ────────────────────────────────────────────────────────
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ── Install config files ────────────────────────────────────────────────────────
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ── Install URDF / xacro ───────────────────────────────────────────────────────
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# ── Install jetson_sensor_bridge ───────────────────────────────────────────────────────
install(PROGRAMS scripts/jetson_sensor_bridge.py
DESTINATION lib/${PROJECT_NAME}
RENAME jetson_sensor_bridge.py)
# ── Install meshes ─────────────────────────────────────────────────────────────
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
)

install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

# ── Linting / Tests ────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Exclude copyright/license checks for rapid development; enable in production.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
