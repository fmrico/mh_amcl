cmake_minimum_required(VERSION 3.5)
project(mh_amcl)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(rclcpp_lifecycle)
find_package(nav2_costmap_2d)
find_package(sensor_msgs)
find_package(visualization_msgs)
find_package(tf2_geometry_msgs)
find_package(geometry_msgs)
find_package(tf2_ros)
find_package(tf2)
find_package(nav2_util)
find_package(nav2_msgs)
find_package(grid_map_msgs)
find_package(grid_map_ros)
find_package(octomap_msgs)
find_package(octomap_ros)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  nav2_costmap_2d
  sensor_msgs
  visualization_msgs
  tf2_geometry_msgs
  geometry_msgs
  tf2_ros
  tf2
  nav2_util
  nav2_msgs
  grid_map_msgs
  grid_map_ros
  octomap_msgs
  octomap_ros
  pcl_conversions
  pcl_ros
)

include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/MH_AMCL.cpp
  src/${PROJECT_NAME}/MapMatcher.cpp
  src/${PROJECT_NAME}/ParticlesDistribution.cpp
  src/${PROJECT_NAME}/LaserCorrecter.cpp
  src/${PROJECT_NAME}/PointCloudCorrecter.cpp
  src/${PROJECT_NAME}/Utils.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(mh_amcl_program
  src/mh_amcl_program.cpp
)
ament_target_dependencies(mh_amcl_program ${dependencies})
target_link_libraries(mh_amcl_program ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mh_amcl_program
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
