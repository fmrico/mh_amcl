cmake_minimum_required(VERSION 3.0.2)
project(aamcl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

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

find_package(GTest REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aamcl
#  CATKIN_DEPENDS costmap_2d geometry_msgs roscpp roslint sensor_msgs tf tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/AAMCL.cpp
  src/${PROJECT_NAME}/ParticlesDistribution.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/aamcl_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

roslint_cpp()
catkin_add_gtest(test_particles tests/test_particles.cpp)
add_dependencies(test_particles ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
if(TARGET test_particles)
  target_link_libraries(test_particles ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${PROJECT_NAME})
endif()
