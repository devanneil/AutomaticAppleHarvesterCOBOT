#include "arm_control_cpp/arm_control.hpp"
#include "arm_control_cpp/robot_state_enum.hpp"
#include "arm_control_cpp/state_machine.hpp"
#include "arm_control_cpp/utils.hpp"

ArmController::ArmController() : Node("arm_controller")
{
    std::string apple_pose_topic = "/" + ARM_NUM + "/apple_locations";
    apple_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        apple_pose_topic,
        10,
        std::bind( &ArmController::appleCallback, this, std::placeholders::_1));
    apple_pose_queue_ = std::deque<geometry_msgs::msg::PoseStamped::SharedPtr>();
    // timer_ = create_wall_timer(
    //     std::chrono::seconds(1),
    //     std::bind(&ArmController::controlLoop, this));
    context_.state = RobotState::Monitor;
    context_.planning_group = "arm_1";
    context_.last_state = std::chrono::steady_clock::now();
    context_.last_qr_scan = context_.last_state;
}

void ArmController::initializeMoveIt() 
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm_1");
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        shared_from_this(),
        "base_link",
        "/rviz_visual_tools",
        move_group_->getRobotModel());

    visual_tools_->loadRemoteControl();
    visual_tools_->deleteAllMarkers();
    visual_tools_->loadTrajectoryPub();

    move_group_->getCurrentPose("suction_link");

    RCLCPP_INFO(
        get_logger(),
        "EEF link: %s",
        move_group_->getEndEffectorLink().c_str());
}

void ArmController::appleCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (!isDuplicatePose(msg))
    {
        apple_pose_queue_.push_back(msg);
    }
    context_.consensus_size++;
}

bool ArmController::moveToPose(
    const geometry_msgs::msg::PoseStamped& target,
    bool blocking = true,
    const std::string end_effector = "suction_link"
    )
{   
    if (!move_group_)
    {
        RCLCPP_WARN(
            get_logger(),
            "MoveGroup not initialized");
        return false;
    }
    auto current_pose = move_group_->getCurrentPose("suction_link");
    if(poseEqual(current_pose, target)) return true;
    move_group_->setPoseTarget(target, end_effector);
    RCLCPP_INFO(get_logger(),
        "Goal pose: x=%f y=%f z=%f",
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z);

    bool success =
        (move_group_->plan(plan) ==
         moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
        RCLCPP_WARN(get_logger(), "GOAL REJECTED!");
        return false;
    }
    else
    {
        if(blocking) visual_tools_->prompt("Execute planned trajectory?");
        return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }
    return false;
}

void ArmController::controlLoop()
{
    if (busy_)
        return;
    if (!move_group_)
    {
        RCLCPP_WARN(
            get_logger(),
            "MoveGroup not initialized");
        return;
    }
    busy_ = true;

    auto current_pose = move_group_->getCurrentPose("suction_link");

    RobotCommand cmd = state_machine_.update(context_);
    RCLCPP_INFO(get_logger(), robotStateToString(context_.state));
    auto pose = cmd.pose;
    RCLCPP_INFO(
        get_logger(),
        "PoseStamped [frame=%s] Pos(%.3f, %.3f, %.3f) Orient(%.3f, %.3f, %.3f, %.3f)",
        pose.header.frame_id.c_str(),
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w);
    pose = move_group_->getCurrentPose("suction_link");
    RCLCPP_INFO(
        get_logger(),
        "PoseStamped [frame=%s] Pos(%.3f, %.3f, %.3f) Orient(%.3f, %.3f, %.3f, %.3f)",
        pose.header.frame_id.c_str(),
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w);

    switch (cmd.type) {
        case CommandType::WaitForUser:
            holdForUser();
            break;
        case CommandType::MoveArm:
            moveToPose(cmd.pose);
            break;
        case CommandType::SelectNextApple:
            context_.target_pose = getNextPose();
            break;
        case CommandType::StartSuction:
            RCLCPP_INFO(get_logger(), "Start suction here!");
            context_.suction_state = true;
            break;
        case CommandType::StopSuction:
            RCLCPP_INFO(get_logger(), "Stop suction here!");
            context_.suction_state = false;
            break;           
        case CommandType::StopArm:
            move_group_->stop();
            break;
        case CommandType::VisionScan:
            RCLCPP_INFO(get_logger(), "Vision scan here!");
            break;
        case CommandType::QRScan:
            RCLCPP_INFO(get_logger(), "QR Scan Here!");
            break;
        default:
            RCLCPP_INFO(get_logger(), "No command specified!");
    }
    // Update context appropriately
    context_.at_pose = poseEqual(move_group_->getCurrentPose("suction_link"), cmd.pose);
    busy_ = false;
}

bool ArmController::isDuplicatePose(const geometry_msgs::msg::PoseStamped::SharedPtr& msg)
{
    constexpr double POS_EPS = 0.01; // 1 cm tolerance

    for (const auto& existing : apple_pose_queue_)
    {
        const auto& a = existing->pose.position;
        const auto& b = msg->pose.position;

        double dx = a.x - b.x;
        double dy = a.y - b.y;
        double dz = a.z - b.z;

        double dist2 = dx*dx + dy*dy + dz*dz;

        if (dist2 < POS_EPS * POS_EPS)
        {
            return true;
        }
    }

    return false;
}
void ArmController::holdForUser() {
    visual_tools_->prompt("Execute next step?");
}
geometry_msgs::msg::PoseStamped ArmController::getNextPose() {
    auto target = apple_pose_queue_.front();
    apple_pose_queue_.pop_front();
    context_.consensus_size--;
    return *target;
}