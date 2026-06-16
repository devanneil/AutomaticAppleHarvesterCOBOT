cmake_minimum_required(VERSION 3.5)
project(apple_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SuctionCommand.srv"
  "msg/AppleConsensus.msg"
  DEPENDENCIES
  std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()