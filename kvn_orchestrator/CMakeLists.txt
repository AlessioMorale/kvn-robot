cmake_minimum_required(VERSION 3.16)
project(kvn_orchestrator LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(explore_lite_msgs REQUIRED)

# The BehaviorTree.CPP CMake target name differs between distributions/builds.
if(TARGET behaviortree_cpp::behaviortree_cpp)
  set(BT_TARGET behaviortree_cpp::behaviortree_cpp)
elseif(TARGET BT::behaviortree_cpp)
  set(BT_TARGET BT::behaviortree_cpp)
else()
  message(FATAL_ERROR "BehaviorTree.CPP was found but exports no known CMake target")
endif()

# Shared infrastructure: ROS context, topic cache, tree host. Plugins link against it so they all
# see the same RosContext type through the blackboard.
add_library(${PROJECT_NAME}_core SHARED
  src/ros_context.cpp
  src/tree_host.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  ${BT_TARGET}
  ament_index_cpp::ament_index_cpp
  nav2_ros_common::nav2_ros_common
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)

# Behaviour-tree node plugins, loaded at runtime by name from the XML-configured plugin list.
function(kvn_add_bt_plugin target)
  add_library(${target} SHARED ${ARGN})
  target_compile_definitions(${target} PRIVATE BT_PLUGIN_EXPORT)
  target_link_libraries(${target} PUBLIC ${PROJECT_NAME}_core)
  install(TARGETS ${target} LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
endfunction()

kvn_add_bt_plugin(kvn_bt_conditions src/plugins/conditions.cpp)
target_link_libraries(kvn_bt_conditions PUBLIC
  tf2::tf2 ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${explore_lite_msgs_TARGETS})

kvn_add_bt_plugin(kvn_bt_actions src/plugins/actions.cpp)
target_link_libraries(kvn_bt_actions PUBLIC tf2::tf2 ${geometry_msgs_TARGETS})

kvn_add_bt_plugin(kvn_bt_control src/plugins/control.cpp)

kvn_add_bt_plugin(kvn_bt_autonomy src/plugins/autonomy.cpp)
target_link_libraries(kvn_bt_autonomy PUBLIC
  tf2::tf2
  nav2_behavior_tree::nav2_behavior_tree
  ${geometry_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
)

add_executable(kvn_orchestrator_node src/orchestrator_main.cpp)
target_link_libraries(kvn_orchestrator_node ${PROJECT_NAME}_core)

install(TARGETS ${PROJECT_NAME}_core LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS kvn_orchestrator_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(
  DIRECTORY config launch trees
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp_action REQUIRED)

  ament_add_gtest(test_autonomy_math test/test_autonomy_math.cpp)
  target_include_directories(test_autonomy_math PRIVATE include)

  ament_add_gtest(test_orchestrator test/test_orchestrator.cpp TIMEOUT 120)
  target_link_libraries(test_orchestrator
    ${PROJECT_NAME}_core
    rclcpp_action::rclcpp_action
    tf2_ros::tf2_ros
    ${nav2_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${explore_lite_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
  )
  target_compile_definitions(test_orchestrator PRIVATE
    KVN_TREE_FILE="${CMAKE_CURRENT_SOURCE_DIR}/trees/kvn_orchestrator.xml"
    KVN_PLUGIN_CONDITIONS="$<TARGET_FILE:kvn_bt_conditions>"
    KVN_PLUGIN_ACTIONS="$<TARGET_FILE:kvn_bt_actions>"
    KVN_PLUGIN_CONTROL="$<TARGET_FILE:kvn_bt_control>"
    KVN_PLUGIN_AUTONOMY="$<TARGET_FILE:kvn_bt_autonomy>"
  )
  add_dependencies(
    test_orchestrator kvn_bt_conditions kvn_bt_actions kvn_bt_control kvn_bt_autonomy)
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME}_core)
ament_package()
