#pragma once

#include <chrono>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "arm_control_cpp/robot_state_enum.hpp"

geometry_msgs::msg::PoseStamped create_pose(
    float x,
    float y,
    float z,
    float roll,
    float pitch,
    float yaw,
    const char* frame_id = "base_link");


template<typename Duration = std::chrono::milliseconds>
Duration duration_since(
    const std::chrono::steady_clock::time_point& start)
{
    return std::chrono::duration_cast<Duration>(
        std::chrono::steady_clock::now() - start);
}

geometry_msgs::msg::PoseStamped twistPick(
    geometry_msgs::msg:PoseStamped pose
);

std::optional<geometry_msgs::msg::PoseStamped> get_pose_for_state(RobotContext &ctx);