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
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Threads REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

add_library(misc SHARED
  src/misc.cpp
  )

add_library(cyclic SHARED
  src/geometry/cyclic.cpp
  )

add_library(geometry_misc SHARED
  src/geometry/misc.cpp
  )

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(TARGETS
  misc
  cyclic
  geometry_misc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include
  DESTINATION .
)

ament_export_include_directories(include)

ament_export_libraries(
  misc
  cyclic
  geometry_misc
)

ament_package()
