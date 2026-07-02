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
    geometry_msgs::msg::PoseStamped pose
);

geometry_msgs::msg::PoseStamped getPoseForState(RobotContext &ctx);

inline const char* robotStateToString(RobotState state)
{
    switch (state)
    {
        case RobotState::Monitor:        return "Monitor";
        case RobotState::Hold:           return "Hold";
        case RobotState::Approach:       return "Approach";
        case RobotState::Pick:           return "Pick";
        case RobotState::Retreat:        return "Retreat";
        case RobotState::QRScan:         return "QRScan";
        case RobotState::QRSearch:       return "QRSearch";
        case RobotState::Chute:          return "Chute";
        case RobotState::ChuteRetreat:   return "ChuteRetreat";
        case RobotState::HeatScan:       return "HeatScan";
        case RobotState::CloseScan:      return "CloseScan";
        case RobotState::Error:          return "Error";
        default:                         return "Unknown";
    }
}

bool poseEqual(geometry_msgs::msg::PoseStamped pose_1, geometry_msgs::msg::PoseStamped pose_2);