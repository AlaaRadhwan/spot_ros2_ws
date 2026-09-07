cmake_minimum_required(VERSION 3.8)
project(spot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    worlds
    config
    scripts
  DESTINATION share/${PROJECT_NAME}
)

install (
  PROGRAMS
    scripts/spot_joint_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()