cmake_minimum_required(VERSION 3.16)
project(kvn_status LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Pure status logic, no ROS dependency
add_library(${PROJECT_NAME}_reducer STATIC
  src/status_reducer.cpp
)
target_include_directories(${PROJECT_NAME}_reducer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
set_target_properties(${PROJECT_NAME}_reducer PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME}_lib STATIC
  src/inputs.cpp
  src/status_node.cpp
)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  ${PROJECT_NAME}_reducer
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
)

add_executable(kvn_status_node src/status_main.cpp)
target_link_libraries(kvn_status_node ${PROJECT_NAME}_lib)

install(TARGETS kvn_status_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_status_reducer test/test_status_reducer.cpp)
  target_link_libraries(test_status_reducer ${PROJECT_NAME}_reducer)

  ament_add_gtest(test_status_node test/test_status_node.cpp TIMEOUT 120)
  target_link_libraries(test_status_node ${PROJECT_NAME}_lib)
endif()

ament_package()
