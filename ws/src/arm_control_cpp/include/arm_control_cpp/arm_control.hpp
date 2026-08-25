#pragma once
#include <memory>
#include <chrono>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include "apple_interfaces/srv/suction_command.hpp"
#include "apple_interfaces/srv/update_bin.hpp"
#include "apple_interfaces/srv/scan_pose_request.hpp"
#include "apple_interfaces/action/vision_scan.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include "utils.hpp"
#include "state_machine.hpp"

class TestArmController;

class ArmController : public rclcpp::Node
{
    friend class TestArmController;
public:
    ArmController();
    ~ArmController();

    void initializeMoveIt();

    void controlLoop();
    bool break_;
    bool holdForUser();
    rclcpp::Client<apple_interfaces::srv::SuctionCommand>::SharedFuture disableVacuum();
private:
    void logPose(const std::string poseContext, const geometry_msgs::msg::PoseStamped& pose);
    //void appleCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void suctionCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void rvizJoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    bool moveToPose(const geometry_msgs::msg::PoseStamped& target, bool blocking, std::string end_effector);
    void stopArm();

    bool cartesianMove(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints, std::string end_effector);

    // void executePlanService(
    //     const std::shared_ptr<rmw_request_id_t> request_header,
    //     const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    //     std::shared_ptr<std_srvs::srv::SetBool::Response> response
    // );
    // void moveToHome(
    //     const std::shared_ptr<rmw_request_id_t> request_header,
    //     const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    //     std::shared_ptr<std_srvs::srv::SetBool::Response> response
    // );

    bool isDuplicatePose(const geometry_msgs::msg::PoseStamped::SharedPtr& msg);

    void triggerVacuum();
    void monitorVacuum();
    bool createVisionScan(uint8_t scan_type);
    bool createAppleScan();
    bool createQRScan();

    using VisionScan = apple_interfaces::action::VisionScan;
    using GoalHandleVisionScan = rclcpp_action::ClientGoalHandle<VisionScan>;

    GoalHandleVisionScan::SharedPtr vision_goal_handle_;
    uint8_t last_goal_order_;

    void goal_response_callback(
        GoalHandleVisionScan::SharedPtr goal_handle);

    void feedback_callback(
        GoalHandleVisionScan::SharedPtr,
        const std::shared_ptr<const VisionScan::Feedback> feedback);

    void result_callback(
        const GoalHandleVisionScan::WrappedResult & result);

    void updateBinPose(geometry_msgs::msg::PoseStamped qr_pose);

    void getScanPose();
    void clearScanPose();

    bool block;

    geometry_msgs::msg::PoseStamped getNextPose();

    rclcpp::CallbackGroup::SharedPtr sub_callback_group_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

    const std::string ARM_NUM = "arm1";
    const uint8_t relay_num = 0;

    std::deque<geometry_msgs::msg::PoseStamped::SharedPtr> apple_pose_queue_;
    geometry_msgs::msg::PoseStamped::SharedPtr home_;

    // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr apple_sub_;
    rclcpp_action::Client<apple_interfaces::action::VisionScan>::SharedPtr camera_client_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    // moveit::planning_interface::MoveGroupInterface::Plan plan;

    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr rviz_gui_sub_;

    rclcpp::Client<apple_interfaces::srv::SuctionCommand>::SharedPtr suction_client_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr suction_sub_;

    rclcpp::Client<apple_interfaces::srv::UpdateBin>::SharedPtr bin_manager_client_;

    rclcpp::Client<apple_interfaces::srv::ScanPoseRequest>::SharedPtr scan_pose_client_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr scan_pose_clear_pub_;

    int vacuum_consensus_count_ = 0;
    uint8_t latest_vacuum_state_ = 0;
    mutable std::mutex context_mutex_;
    bool suction_running_ = false;
    int last_scan_ID = 0;
    rclcpp::TimerBase::SharedPtr vacuum_timer_;
    std::chrono::steady_clock::time_point last_suction_;
    std::chrono::steady_clock::duration suction_timeout_ = std::chrono::seconds(7);
    RobotContext context_;
    StateMachine state_machine_;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    bool busy_ = false;
};