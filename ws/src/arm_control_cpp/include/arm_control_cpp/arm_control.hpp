#pragma once
#include <memory>
#include <chrono>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "apple_interfaces/srv/suction_command.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include "utils.hpp"
#include "state_machine.hpp"

class ArmController : public rclcpp::Node
{
public:
    ArmController();
    ~ArmController();

    void initializeMoveIt();

    void controlLoop();
    bool break_;
    bool holdForUser();
    rclcpp::Client<apple_interfaces::srv::SuctionCommand>::SharedFuture disableVacuum();
private:

    void appleCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void suctionCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void rvizJoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

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

    void triggerVacuum();
    void monitorVacuum();

    bool block;

    geometry_msgs::msg::PoseStamped getNextPose();

    rclcpp::CallbackGroup::SharedPtr sub_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    const std::string ARM_NUM = "arm1";
    const uint8_t relay_num = 0;

    std::deque<geometry_msgs::msg::PoseStamped::SharedPtr> apple_pose_queue_;
    geometry_msgs::msg::PoseStamped::SharedPtr home_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr apple_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    // moveit::planning_interface::MoveGroupInterface::Plan plan;

    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr rviz_gui_sub_;

    rclcpp::Client<apple_interfaces::srv::SuctionCommand>::SharedPtr suction_client_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr suction_sub_;

    int vacuum_consensus_count_ = 0;
    uint8_t latest_vacuum_state_ = 0;
    mutable std::mutex context_mutex_;
    bool suction_running_;
    rclcpp::TimerBase::SharedPtr vacuum_timer_;
    std::chrono::steady_clock::time_point last_suction_;
    std::chrono::steady_clock::duration suction_timeout_ = std::chrono::milliseconds(5000);
    RobotContext context_;
    StateMachine state_machine_;
    bool busy_ = false;
};