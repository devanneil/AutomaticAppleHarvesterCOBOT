#include "arm_control_cpp/state_machine.hpp"

RobotCommand StateMachine::update(RobotContext& ctx) 
{
    RobotCommand nextCommand;
    if (ctx.move_command_fail) {
        // Move command fail handler
        nextCommand.type = CommandType::None;
        nextCommand.requested_state = RobotState::Monitor;
        changeState(ctx, nextCommand.requested_state);
        ctx.move_command_fail = false; // Reset error bit to reeneter control loop
        return nextCommand;
    }
    switch (ctx.state) {
        case RobotState::Monitor:
            nextCommand = handleMonitor(ctx);
            break;
        case RobotState::Hold:
            nextCommand = handleHold(ctx);
            break;
        case RobotState::Approach:
            nextCommand = handleApproach(ctx);
            break;
        case RobotState::Pick:
            nextCommand = handlePick(ctx);
            break;
        case RobotState::Retreat:
            nextCommand = handleRetreat(ctx);
            break;
        case RobotState::QRScan:
            nextCommand = handleQRScan(ctx);
            break;
        case RobotState::QRSearch:
            nextCommand = handleQRSearch(ctx);
            break;
        case RobotState::ChutePrepare:
            nextCommand = handleChutePrepare(ctx);
            break;
        case RobotState::Chute:
            nextCommand = handleChute(ctx);
            break;
        case RobotState::ChuteRetreat:
            nextCommand = handleChuteRetreat(ctx);
            break;
        case RobotState::HeatScan:
            nextCommand = handleHeatScan(ctx);
            break;
        case RobotState::CloseScan:
            nextCommand = handleCloseScan(ctx);
            break;
        case RobotState::Error:
        default:
            throw std::runtime_error("Robot State Error!");
            break;
    }
    changeState(ctx, nextCommand.requested_state);
    return nextCommand;
}

void StateMachine::changeState(RobotContext& ctx, RobotState new_state) 
{
    if (ctx.state == new_state) return;

    ctx.state = new_state;
    ctx.last_state = std::chrono::steady_clock::now();
    ctx.step = 0;
    ctx.general_command_fail = false;
}

/*
Fallback state for any issues, move to monitor for debugging
*/
RobotCommand StateMachine::handleMonitor(RobotContext& ctx) 
{
    if (ctx.state != RobotState::Monitor) throw std::runtime_error("Improper state!");

    if (!ctx.at_pose) {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::MoveArm;
        nextCommand.pose = getPoseForState(ctx);
        nextCommand.requested_state = RobotState::Monitor;
        return nextCommand;
    }
    else 
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::WaitForUser;
        nextCommand.requested_state = RobotState::Hold;
        return nextCommand;
    }
}
/*
Robot home state, where decision tree is selected
*/
RobotCommand StateMachine::handleHold(RobotContext& ctx)
{
    if (ctx.state != RobotState::Hold) throw std::runtime_error("Improper state!");

    if (ctx.suction_state)
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::None;
        nextCommand.requested_state = RobotState::ChutePrepare;
        return nextCommand;
    }
    if (ctx.consensus_size > 0) 
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::SelectNextApple;
        nextCommand.requested_state = RobotState::Approach;
        return nextCommand;
    }
    else
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::None;
        nextCommand.requested_state = RobotState::HeatScan;
        ctx.step = 0;
        return nextCommand;
    }
}

/*
Moving towards pick
*/
RobotCommand StateMachine::handleApproach(RobotContext& ctx)
{
    if (ctx.state != RobotState::Approach) throw std::runtime_error("Improper state!");
    if (!ctx.at_pose)
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::MoveArm;
        nextCommand.pose = ctx.target_pose;
        nextCommand.pose.pose.position.x -= 0.2;
        nextCommand.requested_state = RobotState::Approach;
        return nextCommand;
    }
    else
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::StartSuction;
        nextCommand.pose = ctx.target_pose;
        nextCommand.requested_state = RobotState::Pick;
        return nextCommand;
    }
}

/*
Picking the apple, suction trigger and move to apple directly, merged with twist pose
*/
RobotCommand StateMachine::handlePick(RobotContext& ctx)
{
    if (ctx.state != RobotState::Pick) throw std::runtime_error("Improper state!");
    if (!ctx.at_pose && ctx.step == 0)
    {
        ctx.step = 1;
        RobotCommand nextCommand;
        nextCommand.type = CommandType::MoveArm;
        nextCommand.pose = ctx.target_pose;
        nextCommand.requested_state = RobotState::Pick;
        return nextCommand;
    }
    if (ctx.suction_state && ctx.step == 1)
    {
        ctx.step = 2;
        RobotCommand nextCommand;
        nextCommand.type = CommandType::None;
        nextCommand.requested_state = RobotState::Pick;
        return nextCommand;
    }
    if (ctx.step == 2)
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::MoveArm;
        nextCommand.pose = twistPick(ctx.target_pose);
        nextCommand.requested_state = RobotState::Retreat;
        ctx.step = 0;
        return nextCommand;
    }
    if (timeout_elapsed(ctx.last_state, std::chrono::milliseconds(5000)))
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::StopSuction;
        nextCommand.requested_state = RobotState::Hold;
        ctx.step = 0;
        return nextCommand;
    }
    RobotCommand nextCommand;
    nextCommand.type = CommandType::None;
    nextCommand.requested_state = RobotState::Pick;
    return nextCommand;
}

RobotCommand StateMachine::handleRetreat(RobotContext& ctx)
{
    if (ctx.state != RobotState::Retreat) throw std::runtime_error("Improper state!");
    if(ctx.step == 0 && ctx.at_pose) 
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::MoveArm;
        nextCommand.pose = ctx.target_pose;
        nextCommand.pose.pose.position.x -= 0.2;
        nextCommand.pose.pose.position.z += 0.1;
        nextCommand.requested_state = RobotState::Retreat;
        ctx.step = 1;
        return nextCommand;
    }
    else
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::None;
        nextCommand.requested_state = RobotState::ChutePrepare;
        ctx.step = 0;
        return nextCommand;
    }
}

RobotCommand StateMachine::handleQRScan(RobotContext& ctx)
{
    if (ctx.state != RobotState::QRScan) throw std::runtime_error("Improper state!");
    if (ctx.step == 0)
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::MoveArm;
        nextCommand.pose = getPoseForState(ctx);
        nextCommand.requested_state = RobotState::QRScan;
        ctx.step = 1;
        return nextCommand;
    }
    if (!timeout_elapsed(ctx.last_qr_scan, std::chrono::minutes(3)))
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::None;
        nextCommand.requested_state = RobotState::Chute;
        ctx.step = 0;
        return nextCommand;
    }
    if (timeout_elapsed(ctx.last_state, std::chrono::seconds(10)))
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::MoveArm;
        nextCommand.pose = getPoseForState(ctx);
        nextCommand.requested_state = RobotState::QRSearch;
        ctx.step = 0;
        return nextCommand;
    }
    if (ctx.vision_scan_available) {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::QRScan;
        nextCommand.requested_state = RobotState::QRScan;
        return nextCommand;
    }
    RobotCommand nextCommand;
    nextCommand.type = CommandType::None;
    nextCommand.requested_state = RobotState::QRScan;
    return nextCommand;
}

RobotCommand StateMachine::handleQRSearch(RobotContext& ctx)
{
    if (ctx.state != RobotState::QRSearch) throw std::runtime_error("Improper state!");
    if (!timeout_elapsed(ctx.last_qr_scan, std::chrono::minutes(3)))
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::None;
        nextCommand.requested_state = RobotState::Chute;
        ctx.step = 0;
        return nextCommand;
    }
    if (ctx.vision_scan_available) {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::QRScan;
        nextCommand.requested_state = RobotState::QRSearch;
        return nextCommand;
    }
    RobotCommand nextCommand;
    nextCommand.type = CommandType::None;
    nextCommand.requested_state = RobotState::QRSearch;
    return nextCommand;
}

RobotCommand StateMachine::handleChutePrepare(RobotContext &ctx)
{
    if (ctx.state != RobotState::ChutePrepare) throw std::runtime_error("Improper state!");
    if (timeout_elapsed(ctx.last_qr_scan, std::chrono::minutes(3)))
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::QRScan;
        nextCommand.requested_state = RobotState::QRScan;
        return nextCommand;
    } else
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::None;
        nextCommand.requested_state = RobotState::Chute;
        return nextCommand;
    }
}
RobotCommand StateMachine::handleChute(RobotContext& ctx)
{
    if (ctx.state != RobotState::Chute) throw std::runtime_error("Improper state!");
    if (ctx.step == 0) {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::MoveArm;
        nextCommand.pose = getPoseForState(ctx);
        nextCommand.pose.pose.position.z += 0.05;
        nextCommand.requested_state = RobotState::Chute;
        ctx.step = 2;
        return nextCommand;
    }
    if (ctx.step == 1) {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::MoveArm;
        nextCommand.pose = getPoseForState(ctx);
        nextCommand.requested_state = RobotState::Chute;
        ctx.step = 2;
        return nextCommand;
    }
    if (ctx.step == 2) {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::StopSuction;
        nextCommand.requested_state = RobotState::ChuteRetreat;
        ctx.step = 0;
        return nextCommand;
    }
    RobotCommand nextCommand;
    nextCommand.type = CommandType::None;
    nextCommand.requested_state = RobotState::Chute;
    return nextCommand;
}

RobotCommand StateMachine::handleChuteRetreat(RobotContext& ctx)
{
    if (ctx.state != RobotState::ChuteRetreat) throw std::runtime_error("Improper state!");
    if (ctx.step == 0) {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::MoveArm;
        nextCommand.pose = getPoseForState(ctx);
        nextCommand.pose.pose.position.z += 0.05;
        nextCommand.requested_state = RobotState::ChuteRetreat;
        ctx.step = 1;
        return nextCommand;
    }
    RobotCommand nextCommand;
    nextCommand.type = CommandType::None;
    nextCommand.requested_state = RobotState::Hold;
    return nextCommand;
}

RobotCommand StateMachine::handleHeatScan(RobotContext& ctx)
{
    if (ctx.state != RobotState::HeatScan) throw std::runtime_error("Improper state!");
    if (ctx.general_command_fail)
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::None;
        nextCommand.requested_state = RobotState::Monitor;
        return nextCommand;
    }
    if (ctx.step == 0) {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::GetNextScanPose;
        nextCommand.requested_state = RobotState::HeatScan;
        ctx.step = 1;
        return nextCommand;
    }
    if (ctx.general_command_fail || timeout_elapsed(ctx.last_state, std::chrono::seconds(10)))
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::None;
        nextCommand.requested_state = RobotState::Monitor;
        return nextCommand;
    }
    if (ctx.step == 1) {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::MoveArm;
        nextCommand.pose = ctx.target_pose;
        nextCommand.requested_state = RobotState::HeatScan;
        ctx.step = 2;
        return nextCommand;
    }
    if (ctx.at_pose && ctx.step == 2) {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::None;
        nextCommand.requested_state = RobotState::CloseScan;
        return nextCommand;
    }
    RobotCommand tempCommand;
    tempCommand.type = CommandType::None;
    tempCommand.pose = ctx.target_pose;
    tempCommand.requested_state = RobotState::HeatScan;
    return tempCommand;
}

RobotCommand StateMachine::handleCloseScan(RobotContext& ctx)
{
    if (ctx.state != RobotState::CloseScan) throw std::runtime_error("Improper state!");
    if (timeout_elapsed(ctx.last_state, std::chrono::seconds(10)))
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::None;
        nextCommand.requested_state = RobotState::Hold;
        return nextCommand;
    }
    if (ctx.vision_scan_available)
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::VisionScan;
        nextCommand.requested_state = RobotState::CloseScan;
        return nextCommand;
    }
    if (ctx.consensus_size > 0)
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::None;
        nextCommand.requested_state = RobotState::Hold;
        return nextCommand;
    }
    RobotCommand nextCommand;
    nextCommand.type = CommandType::None;
    nextCommand.requested_state = RobotState::CloseScan;
    return nextCommand;
}