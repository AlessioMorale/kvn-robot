cmake_minimum_required(VERSION 3.16)
project(depthai_descriptions VERSION 3.3.0 LANGUAGES NONE)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.py")
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})


ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
