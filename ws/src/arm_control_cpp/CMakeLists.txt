cmake_minimum_required(VERSION 3.8)
project(arm_control_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(apple_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_library(state_machine
    src/state_machine.cpp
)
target_include_directories(state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(state_machine 
  geometry_msgs
)
add_library(utils
    src/utils.cpp
)
target_include_directories(utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(utils 
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  tf2
)

add_library(arm_control
    src/arm_control.cpp
)

target_link_libraries(arm_control
    state_machine
    utils
)

target_include_directories(arm_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(arm_control 
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  moveit_visual_tools
  sensor_msgs
  geometry_msgs
  std_srvs
  std_msgs
  apple_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(arm_control_cpp
    src/main.cpp
)

target_link_libraries(arm_control_cpp
    arm_control
    state_machine
    utils
)

install(TARGETS
  arm_control_cpp
  DESTINATION lib/${PROJECT_NAME})
install(
    DIRECTORY include/
    DESTINATION include
)


function(add_robot_test TARGET SOURCE)

    add_executable(${TARGET}
        tests/${SOURCE}.cpp
    )

    target_link_libraries(${TARGET}
        arm_control
        state_machine
        utils
    )

    install(TARGETS
        ${TARGET}
        DESTINATION lib/${PROJECT_NAME}
    )

endfunction()

add_robot_test(
    suction_test
    test_suction
)

add_robot_test(
    qr_scan_test
    test_qr_scan
)

add_robot_test(
    pick_test
    test_pick
)

add_robot_test(
    vision_scan_test
    test_vision_scan
)

add_robot_test(
    pick_place_test
    test_pick_place
)

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
