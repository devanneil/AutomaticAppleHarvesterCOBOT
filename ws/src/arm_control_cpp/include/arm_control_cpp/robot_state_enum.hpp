#pragma once

#include <chrono>

#include <geometry_msgs/msg/pose_stamped.hpp>

enum class RobotState
{
    Monitor, // Initial State before data collected
    Hold, // State where decision sub-tree is selected, pick or scan
    Approach, // Move to apple
    Pick, // Suction pick apple
    Retreat, // Move away from apple pose
    QRScan, // Quick QR scan
    QRSearch, // Deep QR scan
    Chute, // Chute frame origin
    ChuteRetreat, // Safely return from chute
    HeatScan, // Same pose as, plan heat search
    CloseScan, // Seach high heat spots
    Error
};

struct RobotContext
{
    RobotState state;

    std::string planning_group;

    geometry_msgs::msg::PoseStamped target_pose;

    int step = 0;
    
    int consensus_size;

    bool suction_state; // true->has apple, false->no apple
    bool at_pose;
    bool vision_scan_fail;
    bool move_command_fail;

    std::chrono::steady_clock::time_point last_state;
    std::chrono::steady_clock::time_point last_qr_scan;

    // current_heat_map

};

enum class CommandType
{
    None,
    WaitForUser, // Prompt user for action
    MoveArm, // Send move arm command to go to pose
    SelectNextApple, // Fill active pose with next apple pose
    StartSuction, // Trigger suction head
    StopSuction, // Disable suction head
    StopArm, // Halt movement
    VisionScan, // Refill consensus list
    QRScan
};

struct RobotCommand
{
    CommandType type = CommandType::None;

    geometry_msgs::msg::PoseStamped pose; // used if MoveArm

    RobotState requested_state;
};