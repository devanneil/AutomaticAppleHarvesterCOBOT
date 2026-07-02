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

void ArmController::executePickSequence()
{
    if (apple_pose_queue_.size() == 0)
        return;
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
    auto target = apple_pose_queue_.front();
    apple_pose_queue_.pop_front();

    auto current_pose = move_group_->getCurrentPose("suction_link");
    home_ = std::make_shared<geometry_msgs::msg::PoseStamped>(current_pose);

    RCLCPP_INFO(get_logger(),
        "Home pose: x=%f y=%f z=%f",
        home_->pose.position.x,
        home_->pose.position.y,
        home_->pose.position.z);

    geometry_msgs::msg::PoseStamped approach = *target;

    approach.pose.position.x -= 0.2;

    RCLCPP_INFO(get_logger(), "Planning approach");

    if(!moveToPose(approach, true)) {
        moveToPose(*home_, true);
        RCLCPP_WARN(get_logger(), "FAILED TO APPROACH");
        return;
    }

    RCLCPP_INFO(get_logger(), "Planning target");

    target->pose.position.x -= 0.1;
    if(!moveToPose(*target, true)) {
        moveToPose(*home_, true);
        RCLCPP_WARN(get_logger(), "FAILED TO TARGET");
        return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Planning retreat");

    if(!moveToPose(approach, false)) {
        moveToPose(*home_, true);
        RCLCPP_WARN(get_logger(), "FAILED TO RETREAT");
        return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Planning home");

    moveToPose(*home_, true);
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