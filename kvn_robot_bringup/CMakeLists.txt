cmake_minimum_required(VERSION 3.16)
project(kvn_robot_bringup LANGUAGES NONE)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY config launch test systemd
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/joy_cmdvel_jitter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_package()
