cmake_minimum_required(VERSION 3.16)
project(kvn_joy_teleop LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(joy_to_twist_node src/joy_to_twist_node.cpp)
target_compile_features(joy_to_twist_node PUBLIC cxx_std_17)
target_link_libraries(joy_to_twist_node PUBLIC
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
)

# Teleop with operator override: replaces teleop_twist_joy and arbitrates /cmd_vel
add_executable(teleop_override_node src/teleop_override_node.cpp)
target_compile_features(teleop_override_node PUBLIC cxx_std_17)
target_include_directories(teleop_override_node PRIVATE include)
target_link_libraries(teleop_override_node PUBLIC
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
  std_msgs::std_msgs
)

install(TARGETS joy_to_twist_node teleop_override_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_command_arbiter test/test_command_arbiter.cpp)
  target_include_directories(test_command_arbiter PRIVATE include)
endif()

ament_package()
