cmake_minimum_required(VERSION 3.24)
project(kvn_display LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED IMPORTED_TARGET libgpiod>=2)

# --- u8g2 (BSD-2-Clause): display drivers and fonts ---
# U8g2_Arduino is u8g2's release repository with just the C library (src/clib). For an offline
# build, pass -DFETCHCONTENT_SOURCE_DIR_U8G2=<a checkout of that tag>.
include(FetchContent)
FetchContent_Declare(u8g2
  URL https://github.com/olikraus/U8g2_Arduino/archive/refs/tags/2.37.1.tar.gz
  URL_HASH SHA256=3b9c6c056b608f995d84296f1753a9f093c9b70cfd1af052509eaba72cdc541b
  # Sources only, built below
  SOURCE_SUBDIR no-cmake-project
)
FetchContent_MakeAvailable(u8g2)
set(U8G2_CLIB ${u8g2_SOURCE_DIR}/src/clib)

# u8g2_fonts.c holds every font (~40 MB): only the fonts the screen uses are built
set(KVN_U8G2_FONTS
  u8g2_font_10x20_tr u8g2_font_9x15_tr u8g2_font_7x13_tr u8g2_font_6x10_tr u8g2_font_5x7_tr)
set(KVN_U8G2_FONTS_C ${CMAKE_CURRENT_BINARY_DIR}/u8g2_kvn_fonts.c)
add_custom_command(
  OUTPUT ${KVN_U8G2_FONTS_C}
  COMMAND Python3::Interpreter ${CMAKE_CURRENT_SOURCE_DIR}/cmake/extract_u8g2_fonts.py
          ${U8G2_CLIB}/u8g2_fonts.c ${KVN_U8G2_FONTS_C} ${KVN_U8G2_FONTS}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cmake/extract_u8g2_fonts.py ${U8G2_CLIB}/u8g2_fonts.c
  COMMENT "Extracting u8g2 fonts"
)

file(GLOB U8G2_SOURCES ${U8G2_CLIB}/*.c)
list(REMOVE_ITEM U8G2_SOURCES ${U8G2_CLIB}/u8g2_fonts.c ${U8G2_CLIB}/u8x8_fonts.c)
add_library(u8g2 STATIC ${U8G2_SOURCES} ${KVN_U8G2_FONTS_C})
target_include_directories(u8g2 SYSTEM PUBLIC ${U8G2_CLIB})
target_compile_options(u8g2 PRIVATE -w)
set_target_properties(u8g2 PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Driver table: every full-buffer setup function declared in u8g2.h, by name
file(STRINGS ${U8G2_CLIB}/u8g2.h U8G2_SETUP_LINES REGEX "^void u8g2_Setup_[A-Za-z0-9_]+_f\\(")
set(U8G2_SETUP_NAMES)
foreach(line IN LISTS U8G2_SETUP_LINES)
  string(REGEX REPLACE "^void u8g2_Setup_([A-Za-z0-9_]+)_f\\(.*" "\\1" name "${line}")
  list(APPEND U8G2_SETUP_NAMES ${name})
endforeach()
list(REMOVE_DUPLICATES U8G2_SETUP_NAMES)
list(SORT U8G2_SETUP_NAMES)
list(LENGTH U8G2_SETUP_NAMES U8G2_SETUP_COUNT)
set(U8G2_SETUP_ENTRIES "")
foreach(name IN LISTS U8G2_SETUP_NAMES)
  string(APPEND U8G2_SETUP_ENTRIES "  {\"${name}\", u8g2_Setup_${name}_f},\n")
endforeach()
configure_file(src/u8g2_setups.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/u8g2_setups.cpp @ONLY)

# --- kvn_display ---
set(KVN_WARNINGS -Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)

# Screen layout, u8g2 display and transports, no ROS dependency
add_library(${PROJECT_NAME}_core STATIC
  src/screen.cpp
  src/display.cpp
  src/transport.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/u8g2_setups.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC u8g2 PkgConfig::GPIOD)
target_compile_options(${PROJECT_NAME}_core PRIVATE ${KVN_WARNINGS})
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(kvn_display_node src/display_main.cpp src/display_node.cpp)
target_link_libraries(kvn_display_node
  ${PROJECT_NAME}_core
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)
target_compile_options(kvn_display_node PRIVATE ${KVN_WARNINGS})

install(TARGETS kvn_display_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_kvn_display test/test_screen.cpp test/test_display.cpp)
  target_link_libraries(test_kvn_display ${PROJECT_NAME}_core)
  target_compile_options(test_kvn_display PRIVATE ${KVN_WARNINGS})
endif()

ament_package()
