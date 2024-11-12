cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

# Set the project name
project(UWBPlugin)

# Find the Ignition libraries
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-sensors6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)

# Source files
set(SOURCES uwb_plugin.cc)

# Header files
set(HEADERS uwb_plugin.hh)

# Build the shared library
add_library(UWBPlugin SHARED ${SOURCES})

# Include directories
target_include_directories(UWBPlugin PUBLIC
  $<INSTALL_INTERFACE:include>
)

# Link the library to the necessary dependencies
target_link_libraries(UWBPlugin
  ignition-gazebo6::ignition-gazebo6
  ignition-sensors6::ignition-sensors6
  ignition-msgs8::ignition-msgs8
  ignition-transport11::ignition-transport11
)

# Install the library
install(TARGETS UWBPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
