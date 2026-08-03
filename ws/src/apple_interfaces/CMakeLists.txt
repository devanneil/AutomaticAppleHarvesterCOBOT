cmake_minimum_required(VERSION 3.5)
project(apple_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SuctionCommand.srv"
  "srv/CloudScan.srv"
  "srv/UpdateBin.srv"
  "srv/ScanPoseRequest.srv"
  "msg/CameraConsensus.msg"
  "action/VisionScan.action"
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()