#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

class ArmController : public rclcpp::Node
{
public:
    ArmController() : Node("arm_controller") {
        apple_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cam_drive_control/apple_location",
            10,
            std::bind( &ArmController::appleCallback, this, std::placeholders::_1));
        timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ArmController::executePickSequence, this));
    }

    void initializeMoveIt() {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_1");
        visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            shared_from_this(),
            "base_link",
            "/rviz_visual_tools",
            move_group_->getRobotModel());

        visual_tools_->loadRemoteControl();
    }

private:
    void appleCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void executePickSequence();

    bool moveToPose(const geometry_msgs::msg::PoseStamped& target);

    geometry_msgs::msg::PoseStamped::SharedPtr last_apple_pose_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr apple_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

    bool busy_ = false;
};

geometry_msgs::msg::PoseStamped create_pose(
        const float x, const float y, const float z, 
        const float qx, const float qy, const float qz, const float qw,
        const char* frame_id = "base_link"
    )
{
    geometry_msgs::msg::PoseStamped pose;

    // Timestamp
    pose.header.stamp = rclcpp::Clock().now();

    // IMPORTANT: set this to your MoveIt planning frame
    pose.header.frame_id = frame_id;

    // Position
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    // Orientation
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;
    pose.pose.orientation.w = qw;

    return pose;
}
void ArmController::appleCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    last_apple_pose_ = msg;
}

bool ArmController::moveToPose(
    const geometry_msgs::msg::PoseStamped& target)
{   
    if (!move_group_)
    {
        RCLCPP_WARN(
            get_logger(),
            "MoveGroup not initialized");
        return false;
    }
    move_group_->setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success =
        (move_group_->plan(plan) ==
         moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
        return false;

    visual_tools_->publishTrajectoryLine(
        plan.trajectory_,
        move_group_->getCurrentState()
            ->getJointModelGroup("arm_1"));

    visual_tools_->trigger();

    return true;
}

void ArmController::executePickSequence()
{
    if (!last_apple_pose_)
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
    busy_ = false;
    auto target = *last_apple_pose_;

    last_apple_pose_.reset();

    //auto home = move_group_->getCurrentPose("tool_frame");
    auto home = create_pose(0.471, 0.274, 1.329, 0.552, -0.445, 0.526, -0.470);

    auto approach = target;

    approach.pose.position.z += 0.01;

    RCLCPP_INFO(get_logger(), "Planning approach");

    moveToPose(approach);

    RCLCPP_INFO(get_logger(), "Planning target");

    moveToPose(target);

    // perform_action();

    RCLCPP_INFO(
        get_logger(),
        "Planning retreat");

    moveToPose(approach);

    RCLCPP_INFO(
        get_logger(),
        "Planning home");

    moveToPose(home);
    busy_ = true;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ArmController>();

    node->initializeMoveIt();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}