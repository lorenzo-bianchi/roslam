cmake_minimum_required(VERSION 3.8)
project(ro_slam_bringup)

find_package(ament_cmake REQUIRED)

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
