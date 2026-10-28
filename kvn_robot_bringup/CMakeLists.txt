cmake_minimum_required(VERSION 3.16)
project(kvn_robot_bringup LANGUAGES NONE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# robot_config: reads config/robot.yaml for the launch files
ament_python_install_package(${PROJECT_NAME})

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
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_robot_config test/test_robot_config.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
