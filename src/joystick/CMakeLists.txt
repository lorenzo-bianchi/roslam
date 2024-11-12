cmake_minimum_required(VERSION 3.8)
project(joystick)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ro_slam_interfaces REQUIRED)
find_package(Threads REQUIRED)

# Configure Joystick node source files
file(GLOB JOYSTICK_NODE_SRCS
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  "src/joystick/*.cpp")

# Joystick node
add_library(joystick SHARED ${JOYSTICK_NODE_SRCS})
target_compile_definitions(joystick PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(joystick PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joystick PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_link_libraries(joystick
  ${OpenCV_LIBS}
  Threads::Threads)
ament_target_dependencies(joystick
  rclcpp
  rclcpp_components
  ro_slam_interfaces)
rclcpp_components_register_nodes(joystick "joystick::JoystickNode")

# Joystick standalone application
add_executable(joystick_app
  src/joystick_app.cpp)
target_include_directories(joystick_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joystick_app PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_link_libraries(joystick_app joystick)
ament_target_dependencies(joystick_app rclcpp)

# Install Joystick component
install(TARGETS joystick
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install Joystick app
install(TARGETS joystick_app
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
