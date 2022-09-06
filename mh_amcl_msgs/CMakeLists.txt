project(mh_amcl_msgs)

cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HypoInfo.msg"
  "msg/Info.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs nav2_msgs
)

ament_export_dependencies(rosidl_default_runtime builtin_interfaces geometry_msgs nav2_msgs)
ament_package()
