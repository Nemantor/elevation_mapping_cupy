cmake_minimum_required(VERSION 3.5)
project(elevation_map_msgs)

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)



## Generate services in the 'srv' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ChannelInfo.msg"
  "msg/Statistics.msg"
  DEPENDENCIES std_msgs geometry_msgs
 )

ament_package()