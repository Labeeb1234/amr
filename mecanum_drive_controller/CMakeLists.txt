cmake_minimum_required(VERSION 3.8)
project(mecanum_drive_controller)

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   # we dont use add_compile_options with pedantic in message packages
#   # because the Python C extensions dont comply with it
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
# endif()

# if(WIN32)
#   add_compile_definitions(
#     # For math constants
#     _USE_MATH_DEFINES
#     # Minimize Windows namespace collision
#     NOMINMAX
#     WIN32_LEAN_AND_MEAN
#   )
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

option(ENABLE_PROFILER "Enable Ignition Profiler" FALSE)
if(ENABLE_PROFILER)
  find_package(ignition-common3 QUIET COMPONENTS profiler)
  if(ignition-common3_FOUND)
    add_definitions("-DIGN_PROFILER_ENABLE=1" "-DIGN_PROFILER_REMOTERY=1")
    message(STATUS "Profiler is active")
  else()
    message(WARNING "Can't find Ignition common3. Profiler will not be actived")
    add_definitions("-DIGN_PROFILER_ENABLE=0" "-DIGN_PROFILER_REMOTERY=0")
  endif()
endif()


# gazebo_ros_diff_drive
add_library(mecanum_drive_controller SHARED
  src/mecanum_drive_controller.cpp
)
target_include_directories(mecanum_drive_controller PUBLIC include)
ament_target_dependencies(mecanum_drive_controller
  gazebo_dev
  gazebo_ros
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)
ament_export_libraries(mecanum_drive_controller)
if(ENABLE_PROFILER)
  target_include_directories(mecanum_drive_controller PUBLIC ${ignition-common3_INCLUDE_DIRS})
  target_link_libraries(mecanum_drive_controller ${ignition-common3_LIBRARIES})
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
