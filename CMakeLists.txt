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

if ("$ENV{ROS_DISTRO}" STREQUAL "galactic")
  add_definitions("-DROS_GALACTIC")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Threads REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                       compile                              |
## --------------------------------------------------------------

set(dependencies
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

add_library(misc SHARED
  src/misc.cpp
  )

add_library(scope_timer SHARED
  src/scope_timer.cpp
  )

ament_target_dependencies(scope_timer rclcpp)

add_library(median_filter SHARED
  src/median_filter.cpp
  )

add_library(cyclic SHARED
  src/geometry/cyclic.cpp
  )

add_library(geometry_misc SHARED
  src/geometry/misc.cpp
  )

ament_target_dependencies(geometry_misc
  geometry_msgs
  tf2
  tf2_geometry_msgs
  )
## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(TARGETS
  misc
  scope_timer
  median_filter
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
  scope_timer
  median_filter
  cyclic
  geometry_misc
)
ament_export_dependencies(${dependencies})

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

if(BUILD_TESTING)
  # Include linting tests
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(geometry_test test/geometry/geometry_tests.cpp)
  ament_target_dependencies(geometry_test geometry_msgs tf2 tf2_geometry_msgs)
  ament_add_gtest(median_filter_test test/median_filter/tests.cpp src/median_filter.cpp)

endif()

ament_package()
