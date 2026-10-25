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

# Executable copy next to the units (the DIRECTORY install above drops the exec bit)
install(
  PROGRAMS systemd/install.sh
  DESTINATION share/${PROJECT_NAME}/systemd
)

if(BUILD_TESTING)
endif()

ament_package()
