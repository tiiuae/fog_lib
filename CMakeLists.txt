cmake_minimum_required(VERSION 3.12)
project(fog_lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Wpedantic")
add_definitions("-g")
add_definitions("-O3")
if ("$ENV{ROS_DISTRO}" STREQUAL "foxy") # to select the correct version of declare_parameter() and avoid errors/warnings
  add_definitions("-DROS_FOXY")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Threads REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

add_library(fog_lib SHARED
  src/fog_lib.cpp
  )

target_compile_definitions(fog_lib
  PRIVATE "${PROJECT_NAME}_BUILDING_DLL")

ament_target_dependencies(fog_lib
  rclcpp
  rclcpp_components
  rcl_interfaces
  Threads
  )

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(TARGETS
  fog_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include
  DESTINATION .
)

ament_export_include_directories(include)

ament_package()
