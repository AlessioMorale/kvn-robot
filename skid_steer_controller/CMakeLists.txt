cmake_minimum_required(VERSION 3.16)
project(skid_steer_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(skid_steer_controller SHARED
  src/skid_steer_controller.cpp
)

target_include_directories(skid_steer_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/skid_steer_controller>
)

target_link_libraries(skid_steer_controller PUBLIC
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Disable boost for pluginlib
target_compile_definitions(skid_steer_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Export the plugin
pluginlib_export_plugin_description_file(controller_interface plugins.xml)

install(
  TARGETS skid_steer_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  FILES include/skid_steer_controller/skid_steer_controller.hpp
  DESTINATION include/skid_steer_controller
)

install(
  FILES plugins.xml
  DESTINATION share/skid_steer_controller
)

ament_package()

