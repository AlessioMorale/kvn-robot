cmake_minimum_required(VERSION 3.16)
project(kvn_robot_description LANGUAGES NONE)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf test
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_kvn_rover_urdf_xacro test/kvn_rover_test_urdf_xacro.py)
endif()

ament_package()
