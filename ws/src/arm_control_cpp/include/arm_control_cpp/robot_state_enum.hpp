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
    ChutePrepare, // Non movement prechute step, decide to QR scan or not
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

    bool suction_state = false; // true->has apple, false->no apple
    bool at_pose = false;
    bool vision_scan_available = true;
    bool move_command_fail = false;
    bool general_command_fail = false;

    std::chrono::steady_clock::time_point last_state;
    std::chrono::steady_clock::time_point last_qr_scan;

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
    QRScan,
    GetNextScanPose,
    CartesianMove
};

struct RobotCommand
{
    CommandType type = CommandType::None;

    geometry_msgs::msg::PoseStamped pose; // used if MoveArm
    std::vector<geometry_msgs::msg::PoseStamped> waypoints; // used if CartesianMove

    RobotState requested_state;
};