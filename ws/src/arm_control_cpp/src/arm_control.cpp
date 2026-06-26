#include <memory>
#include <chrono>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

const std::string ARM_NUM = "arm1";
class ArmController : public rclcpp::Node
{
public:
    ArmController() : Node("arm_controller") {
        std::string apple_pose_topic = "/" + ARM_NUM + "/apple_locations";
        apple_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            apple_pose_topic,
            10,
            std::bind( &ArmController::appleCallback, this, std::placeholders::_1));
        apple_pose_queue_ = std::deque<geometry_msgs::msg::PoseStamped::SharedPtr>();
        // timer_ = create_wall_timer(
        //     std::chrono::seconds(1),
        //     std::bind(&ArmController::executePickSequence, this));
        // execute_service_ =
        //     create_service<std_srvs::srv::SetBool>(
        //         "execute_plan",
        //         std::bind(
        //             &ArmController::executePlanService,
        //             this,
        //             std::placeholders::_1,
        //             std::placeholders::_2,
        //             std::placeholders::_3));
        // home_service_ =
        //     create_service<std_srvs::srv::SetBool>(
        //         "move_to_home",
        //         std::bind(
        //             &ArmController::moveToHome,
        //             this,
        //             std::placeholders::_1,
        //             std::placeholders::_2,
        //             std::placeholders::_3));
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
        visual_tools_->deleteAllMarkers();
        visual_tools_->loadTrajectoryPub();

        move_group_->getCurrentPose("suction_link");

        RCLCPP_INFO(
            get_logger(),
            "EEF link: %s",
            move_group_->getEndEffectorLink().c_str());

    }

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

    std::deque<geometry_msgs::msg::PoseStamped::SharedPtr> apple_pose_queue_;
    geometry_msgs::msg::PoseStamped::SharedPtr home_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr apple_sub_;
    //rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr execute_service_;
    //rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr home_service_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

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
    //auto result = move_group_->execute(plan);

    // if (result != moveit::core::MoveItErrorCode::SUCCESS)
    // {
    //     RCLCPP_ERROR(get_logger(), "Execution failed!");
    //     return false;
    // }

    // const auto& first =
    //     plan.trajectory_.joint_trajectory.points.front();

    // const auto& last =
    //     plan.trajectory_.joint_trajectory.points.back();

    // for (size_t i = 0; i < first.positions.size(); ++i)
    // {
    //     RCLCPP_INFO(get_logger(),
    //         "Joint %zu: start=%f end=%f",
    //         i,
    //         first.positions[i],
    //         last.positions[i]);
    // }

    return false;
}
// void ArmController::executePlanService(
//         const std::shared_ptr<rmw_request_id_t> request_header,
//         const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
//         std::shared_ptr<std_srvs::srv::SetBool::Response> response
//     )
// {
//     if (request->data) {
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received: TRUE");
//         move_group_->execute(plan);
//         response->success = true;
//         response->message = "Boolean was TRUE, action completed.";
//     } else {
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received: FALSE");
//         response->success = false;
//         response->message = "Boolean was FALSE, no action taken.";
//     }
// }
// void ArmController::moveToHome(
//         const std::shared_ptr<rmw_request_id_t> request_header,
//         const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
//         std::shared_ptr<std_srvs::srv::SetBool::Response> response
//     ) {
//         if (request->data) {
//             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received: TRUE");
//             moveToPose(*home_);
//             response->message = "Boolean was TRUE, action completed.";
//         } else {
//             RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received: FALSE");
//             moveToPose(*home_);
//         }
//     }
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

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ArmController>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);

    auto spinner = std::thread([&executor]() {executor.spin(); });

    node->initializeMoveIt();

    while (rclcpp::ok())
    {
        node->executePickSequence();

        std::this_thread::sleep_for(
            std::chrono::milliseconds(100));
    }
    spinner.join();

    rclcpp::shutdown();

    return 0;
}

// int main(int argc, char * argv[])
// {
//     // Initialize ROS and create the Node
//     rclcpp::init(argc, argv);
//     auto const node = std::make_shared<rclcpp::Node>(
//     "hello_moveit",
//     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//     );

//     // Create a ROS logger
//     auto const logger = rclcpp::get_logger("hello_moveit");

//     // Create the MoveIt MoveGroup Interface
//     using moveit::planning_interface::MoveGroupInterface;
//     auto move_group_interface = MoveGroupInterface(node, "arm_1");

//     // Set a target Pose
//     auto const target_pose = []{
//     geometry_msgs::msg::Pose msg;
//     msg.orientation.w = 1.0;
//     msg.position.x = 0.28;
//     msg.position.y = -0.2;
//     msg.position.z = 0.5;
//     return msg;
//     }();
//     move_group_interface.setPoseTarget(target_pose);
//     move_group_interface.getCurrentPose("suction_link");
//     // Create a plan to that target pose
//     auto const [success, plan] = [&move_group_interface]{
//     moveit::planning_interface::MoveGroupInterface::Plan msg;
//     auto const ok = static_cast<bool>(move_group_interface.plan(msg));
//     return std::make_pair(ok, msg);
//     }();

//     // Execute the plan
//     if(success) {
//         //move_group_interface.execute(plan);
//         RCLCPP_INFO(logger, "Planning Success!");
//     } else {
//         RCLCPP_ERROR(logger, "Planing failed!");
//     }

//     // Shutdown ROS
//     rclcpp::shutdown();
//     return 0;
// }