cmake_minimum_required(VERSION 3.12)
project(rbpodo_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_rb5_joint_limits test/test_rb5_joint_limits.py)
endif()

install(DIRECTORY meshes robots launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
