#pragma once
#include <memory>
#include <chrono>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

class ArmController : public rclcpp::Node
{
public:
    ArmController();

    void initializeMoveIt();

    void executePickSequence();

private:
    void appleCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    bool moveToPose(const geometry_msgs::msg::PoseStamped& target, bool blocking, std::string end_effector);

    void executePlanService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response
    );
    void moveToHome(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response
    );
    bool isDuplicatePose(const geometry_msgs::msg::PoseStamped::SharedPtr& msg);

    const std::string ARM_NUM = "arm1";

    std::deque<geometry_msgs::msg::PoseStamped::SharedPtr> apple_pose_queue_;
    geometry_msgs::msg::PoseStamped::SharedPtr home_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr apple_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

    bool busy_ = false;
};