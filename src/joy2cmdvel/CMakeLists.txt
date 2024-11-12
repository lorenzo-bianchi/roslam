cmake_minimum_required(VERSION 3.15)
project(joy2cmdvel)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

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
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ro_slam_interfaces REQUIRED)

# Configure Joy2cmdvel node source files
file(GLOB JOY2CMDVEL_NODE_SRCS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/joy2cmdvel/*.cpp")

# Joy2cmdvel node
add_library(joy2cmdvel SHARED ${JOY2CMDVEL_NODE_SRCS})
target_compile_definitions(joy2cmdvel PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(joy2cmdvel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joy2cmdvel PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_link_libraries(joy2cmdvel
  ${OpenCV_LIBS}
  Threads::Threads)
ament_target_dependencies(joy2cmdvel
  geometry_msgs
  rclcpp
  rclcpp_components
  ro_slam_interfaces)
rclcpp_components_register_nodes(joy2cmdvel "Joy2cmdvel::Joy2cmdvelNode")

# Joy2cmdvel standalone application
add_executable(joy2cmdvel_app
  src/joy2cmdvel_app.cpp)
target_include_directories(joy2cmdvel_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joy2cmdvel_app PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_link_libraries(joy2cmdvel_app joy2cmdvel)
ament_target_dependencies(joy2cmdvel_app rclcpp)

# Install Joy2cmdvel component
install(TARGETS joy2cmdvel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install Joy2cmdvel app
install(TARGETS joy2cmdvel_app
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

ament_package()