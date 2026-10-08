cmake_minimum_required(VERSION 3.8)
project(kvn_video_streamer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(foxglove_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
find_package(ament_cmake_gtest QUIET)

# Pure, ROS-free helpers (H.264 inspection, pipeline description).
add_library(${PROJECT_NAME}_core STATIC
  src/h264_inspect.cpp
  src/pipeline_config.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME}_lib STATIC
  src/video_streamer_node.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
# GStreamer headers are third-party: keep -Werror=conversion off them.
target_include_directories(${PROJECT_NAME}_lib SYSTEM PUBLIC ${GST_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  ${PROJECT_NAME}_core
  rclcpp::rclcpp
  ${foxglove_msgs_TARGETS}
  PkgConfig::GST)

add_executable(video_streamer_node src/video_streamer_main.cpp)
target_link_libraries(video_streamer_node ${PROJECT_NAME}_lib)

install(TARGETS video_streamer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(${PROJECT_NAME}_test_h264_inspect test/test_h264_inspect.cpp)
    if(TARGET ${PROJECT_NAME}_test_h264_inspect)
      target_link_libraries(${PROJECT_NAME}_test_h264_inspect ${PROJECT_NAME}_core)
    endif()

    ament_add_gtest(${PROJECT_NAME}_test_pipeline_config test/test_pipeline_config.cpp)
    if(TARGET ${PROJECT_NAME}_test_pipeline_config)
      target_link_libraries(${PROJECT_NAME}_test_pipeline_config ${PROJECT_NAME}_core)
    endif()

    # Runs the node with videotestsrc + x264enc and checks the stream.
    ament_add_gtest(${PROJECT_NAME}_test_stream test/test_stream.cpp TIMEOUT 120)
    if(TARGET ${PROJECT_NAME}_test_stream)
      target_link_libraries(${PROJECT_NAME}_test_stream ${PROJECT_NAME}_lib)
    endif()
  endif()
endif()

ament_package()
