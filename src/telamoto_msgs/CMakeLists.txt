cmake_minimum_required(VERSION 3.22)
project(telamoto_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/JogAxes.msg"
  "srv/MoveToNamedPose.srv"
  "srv/SetAutonomous.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_package()
