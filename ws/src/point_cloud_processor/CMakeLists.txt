cmake_minimum_required(VERSION 3.8)
project(point_cloud_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(apple_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
find_package(tf2_eigen REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(ENABLE_PIPE_TIMING)
endif()
add_executable(point_cloud_processor src/point_cloud_processor.cpp)
ament_target_dependencies(point_cloud_processor 
  rclcpp
  apple_interfaces
  geometry_msgs
  sensor_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
  OpenCV
  Eigen
  tf2_eigen
)
install(TARGETS
  point_cloud_processor
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
