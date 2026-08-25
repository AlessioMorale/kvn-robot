cmake_minimum_required(VERSION 3.16)
project(kvn_ekf LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

ament_package()
