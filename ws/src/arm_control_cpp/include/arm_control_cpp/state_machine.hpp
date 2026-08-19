#pragma once

#include "arm_control_cpp/robot_state_enum.hpp"
#include "arm_control_cpp/utils.hpp"

// enum class RobotState
// {
//     Monitor, // Initial State before data collected
//     Hold, // Data collected
//     Approach, // Move to apple
//     Pick, // Suction pick apple
//     Retreat, // Move away from apple pose
//     QRScan, // Quick QR scan
//     QRSearch, // Deep QR scan
//     Chute, // Chute frame origin
//     ChuteRetreat, // Safely return from chute
//     HeatScan, // Same pose as, plan heat search
//     CloseScan // Seach high heat spots
// };
class StateMachine
{
public:
    StateMachine() = default;

    // Main update loop
    RobotCommand update(RobotContext& ctx);

    void changeState(RobotContext& ctx, RobotState new_state);

private:

    RobotCommand handleMonitor(RobotContext& ctx);
    RobotCommand handleHold(RobotContext& ctx);
    // RobotCommand handleApproach(RobotContext& ctx);
    RobotCommand handlePick(RobotContext& ctx);
    // RobotCommand handleRetreat(RobotContext& ctx);
    RobotCommand handleQRScan(RobotContext& ctx);
    RobotCommand handleQRSearch(RobotContext& ctx);
    RobotCommand handleChutePrepare(RobotContext& ctx);
    RobotCommand handleChute(RobotContext& ctx);
    RobotCommand handleChuteRetreat(RobotContext& ctx);
    RobotCommand handleHeatScan(RobotContext& ctx);
    RobotCommand handleCloseScan(RobotContext& ctx);
};