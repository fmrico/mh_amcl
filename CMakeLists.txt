cmake_minimum_required(VERSION 3.0.2)
project(aamcl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE debug)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  roscpp
  roslint
  sensor_msgs
  tf
  tf2
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aamcl
#  CATKIN_DEPENDS costmap_2d geometry_msgs roscpp roslint sensor_msgs tf tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/AAMCL.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/aamcl_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

roslint_cpp()
catkin_add_gtest(test1 tests/test1.cpp)
add_dependencies(test1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
if(TARGET test1)
  target_link_libraries(test1 ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()
