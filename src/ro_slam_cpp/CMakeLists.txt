cmake_minimum_required(VERSION 3.8)
project(ro_slam_cpp)

# set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ro_slam_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

# Configure RO-SLAM node source files
file(GLOB RO_SLAM_NODE_SRCS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/ro_slam_cpp/*.cpp")

# ROSlam node
add_library(ro_slam_cpp SHARED ${RO_SLAM_NODE_SRCS})
target_compile_definitions(ro_slam_cpp PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(ro_slam_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ro_slam_cpp
  Eigen3::Eigen)
ament_target_dependencies(ro_slam_cpp
  rclcpp
  rclcpp_components
  ro_slam_interfaces
  sensor_msgs)
rclcpp_components_register_nodes(ro_slam_cpp "ROSlam::ROSlamNode")

# RO-Slam standalone application
add_executable(ro_slam_cpp_app
  src/ro_slam_app.cpp)
target_include_directories(ro_slam_cpp_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ro_slam_cpp_app ro_slam_cpp)
ament_target_dependencies(ro_slam_cpp_app
  rclcpp)

# Install ROSlam component
install(TARGETS ro_slam_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install ROSlam app
install(TARGETS ro_slam_cpp_app
  DESTINATION lib/${PROJECT_NAME})

# Install Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Install Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Make sure that the library path is exported even if the library itself is not
ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")

ament_package()
