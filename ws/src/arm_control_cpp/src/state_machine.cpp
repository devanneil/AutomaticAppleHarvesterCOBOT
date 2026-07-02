#include "arm_control_cpp/state_machine.hpp"

RobotCommand StateMachine::update(RobotContext& ctx) 
{
    RobotCommand nextCommand;
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
        nextCommand.type = CommandType::MoveArm;
        nextCommand.pose = getPoseForState(ctx);
        nextCommand.requested_state = RobotState::HeatScan;
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
    if (!ctx.at_pose && ctx.step != 1)
    {
        ctx.step = 0;
        RobotCommand nextCommand;
        nextCommand.type = CommandType::MoveArm;
        nextCommand.pose = ctx.target_pose;
        nextCommand.requested_state = RobotState::Pick;
        return nextCommand;
    }
    if (ctx.suction_state && ctx.step != 1)
    {
        ctx.step = 1;
        RobotCommand nextCommand;
        nextCommand.type = CommandType::StopArm;
        nextCommand.requested_state = RobotState::Pick;
        return nextCommand;
    }
    if (ctx.step == 1)
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::MoveArm;
        nextCommand.pose = twistPick(ctx.target_pose);
        nextCommand.requested_state = RobotState::Retreat;
        return nextCommand;
    }
    if (duration_since(ctx.last_state) > std::chrono::milliseconds(500))
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::StopSuction;
        nextCommand.requested_state = RobotState::Monitor;
        return nextCommand;
    }
}

RobotCommand StateMachine::handleRetreat(RobotContext& ctx)
{
    if (ctx.state != RobotState::Retreat) throw std::runtime_error("Improper state!");
    if (!ctx.at_pose)
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::MoveArm;
        nextCommand.pose.pose.position.x -= 0.2;
        nextCommand.requested_state = RobotState::Retreat;
        return nextCommand;
    }
    else
    {
        RobotCommand nextCommand;
        nextCommand.type = CommandType::None;
        nextCommand.requested_state = RobotState::Monitor;
        return nextCommand;
    }
}

RobotCommand StateMachine::handleQRScan(RobotContext& ctx)
{
    if (ctx.state != RobotState::QRScan) throw std::runtime_error("Improper state!");
}

RobotCommand StateMachine::handleQRSearch(RobotContext& ctx)
{
    if (ctx.state != RobotState::QRSearch) throw std::runtime_error("Improper state!");
}

RobotCommand StateMachine::handleChute(RobotContext& ctx)
{
    if (ctx.state != RobotState::Chute) throw std::runtime_error("Improper state!");
}

RobotCommand StateMachine::handleChuteRetreat(RobotContext& ctx)
{
    if (ctx.state != RobotState::ChuteRetreat) throw std::runtime_error("Improper state!");
}

RobotCommand StateMachine::handleHeatScan(RobotContext& ctx)
{
    if (ctx.state != RobotState::HeatScan) throw std::runtime_error("Improper state!");
    RobotCommand tempCommand;
    tempCommand.requested_state = RobotState::Monitor;
    return tempCommand;
}

RobotCommand StateMachine::handleCloseScan(RobotContext& ctx)
{
    if (ctx.state != RobotState::CloseScan) throw std::runtime_error("Improper state!");
}