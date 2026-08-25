cmake_minimum_required(VERSION 3.16)
project(kvn_joy_teleop LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(joy_to_twist_node src/joy_to_twist_node.cpp)
target_compile_features(joy_to_twist_node PUBLIC cxx_std_17)
target_link_libraries(joy_to_twist_node PUBLIC
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
)

install(TARGETS joy_to_twist_node DESTINATION lib/${PROJECT_NAME})

ament_package()
