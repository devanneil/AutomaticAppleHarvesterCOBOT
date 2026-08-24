#pragma once

#include <chrono>
#include <optional>
#include <cmath> 

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "arm_control_cpp/robot_state_enum.hpp"

double degreesToRadians(double degrees);
double radiansToDegrees(double radians);

geometry_msgs::msg::PoseStamped create_pose(
    float x,
    float y,
    float z,
    float roll,
    float pitch,
    float yaw,
    const char* frame_id = "base_link");


template<typename Duration = std::chrono::milliseconds>
bool timeout_elapsed(
    const std::chrono::steady_clock::time_point& start,
    Duration timeout)
{
    if (start == std::chrono::steady_clock::time_point::max())
    {
        return true;
    }
    if (start == std::chrono::steady_clock::time_point::min())
    {
        return true;
    }

    return std::chrono::steady_clock::now() - start >= timeout;
}

geometry_msgs::msg::PoseStamped twistPick(
    geometry_msgs::msg::PoseStamped pose,
    double twist_angle = 20
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
        case RobotState::ChutePrepare:   return "ChutePrepate";
        case RobotState::HeatScan:       return "HeatScan";
        case RobotState::CloseScan:      return "CloseScan";
        case RobotState::Error:          return "Error";
        default:                         return "Unknown";
    }
}

bool poseEqual(const geometry_msgs::msg::PoseStamped &pose_1, const geometry_msgs::msg::PoseStamped &pose_2);