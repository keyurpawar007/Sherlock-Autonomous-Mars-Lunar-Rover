cmake_minimum_required(VERSION 3.8)
project(rover_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf launch config rviz worlds
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(
  PROGRAMS
    scripts/wheel_odometry.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
