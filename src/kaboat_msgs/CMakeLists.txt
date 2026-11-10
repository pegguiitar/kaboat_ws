cmake_minimum_required(VERSION 3.12)
project(kaboat_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/Mark.msg"
  "msg/MarkArray.msg"
  "msg/Buoy.msg"
  "msg/BuoyArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
