cmake_minimum_required(VERSION 3.8)
project(kvn_ekf)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(fuse_core REQUIRED)
find_package(fuse_graphs REQUIRED)
find_package(fuse_models REQUIRED)
find_package(fuse_optimizers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

ament_package()
