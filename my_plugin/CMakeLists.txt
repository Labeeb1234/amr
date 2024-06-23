cmake_minimum_required(VERSION 3.8)
project(my_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(OGRE OGRE)
    pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



link_directories(${GAZEBO_LIBRARY_DIRS} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} ${OGRE_LIBRARY_DIRS})
include_directories(${OpenCV_INCLUDE_DIR}
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${OGRE_INCLUDE_DIRS}
                    ${OGRE-Terrain_INCLUDE_DIRS}
                    include)



################## 2. A simple model controller for the quadrotor #############
add_library( my_plugin SHARED
  src/my_plugin.cpp
include/my_plugin/my_plugin.hpp
)


target_compile_features(my_plugin PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(my_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(my_plugin
  "rclcpp"
  "gazebo_ros"
  "gazebo_plugins"
  "gazebo_dev"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)


# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(my_plugin PRIVATE "MY_PLUGIN_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS my_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


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

set_target_properties(my_plugin
    PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
    ${PROJECT_SOURCE_DIR}/plugins
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  my_plugin
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
