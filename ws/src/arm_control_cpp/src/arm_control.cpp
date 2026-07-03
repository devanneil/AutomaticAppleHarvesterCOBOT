#include "arm_control_cpp/arm_control.hpp"
#include "arm_control_cpp/robot_state_enum.hpp"
#include "arm_control_cpp/state_machine.hpp"
#include "arm_control_cpp/utils.hpp"

ArmController::ArmController() : Node("arm_controller")
{
    sub_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    timer_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = sub_callback_group_;
    std::string apple_pose_topic = "/" + ARM_NUM + "/apple_locations";
    apple_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        apple_pose_topic,
        10,
        std::bind( &ArmController::appleCallback, this, std::placeholders::_1),
        sub_options
    );
    apple_pose_queue_ = std::deque<geometry_msgs::msg::PoseStamped::SharedPtr>();
    suction_sub_ = create_subscription<std_msgs::msg::UInt8>(
        "/suction_state",
        10,
        std::bind(&ArmController::suctionCallback, this, std::placeholders::_1),
        sub_options
    );
    rviz_gui_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "/rviz_visual_tools_gui",
        10,
        std::bind(&ArmController::rvizJoyCallback, this, std::placeholders::_1),
        sub_options
    );
    suction_client_ = create_client<apple_interfaces::srv::SuctionCommand>("/suction_action");
    vacuum_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ArmController::monitorVacuum, this),
        timer_callback_group_
    );
    // timer_ = create_wall_timer(
    //     std::chrono::milliseconds(100),
    //     std::bind(&ArmController::controlLoop, this),
    //     timer_callback_group_
    // );
    context_.state = RobotState::Monitor;
    context_.planning_group = "arm_1";
    context_.last_state = std::chrono::steady_clock::now();
    context_.last_qr_scan = context_.last_state;
}
ArmController::~ArmController() {
    disableVacuum();
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
        msg->pose.position.z += 0.02;
        msg->pose.position.y -= 0.02;
        apple_pose_queue_.push_back(msg);
    }
    {
        std::lock_guard<std::mutex> ctx_lock(context_mutex_);
        context_.consensus_size++;
    }
}
void ArmController::suctionCallback(
    const std_msgs::msg::UInt8::SharedPtr msg)
{
    latest_vacuum_state_ = msg->data;
    uint8_t mask = (16 << relay_num); // Last 4 bits of integer
    {
        std::lock_guard<std::mutex> lock(context_mutex_);
        context_.suction_state = (latest_vacuum_state_ & mask);
    }
}
void ArmController::rvizJoyCallback(
    const sensor_msgs::msg::Joy::SharedPtr msg
)
{
    auto buttons = msg->buttons;
    if (buttons[1] == 1) {
        block = false;
        return; //Next 
    }
    if (buttons[2] == 1) {
        block = false;
        return; //Continue
    }
    if (buttons[3] == 1) {
        RCLCPP_INFO(get_logger(), "Safely stopping node");
        break_ = true;
        return;
    }
    if (buttons[4] == 1) {
        RCLCPP_INFO(get_logger(), "Safely stopping node");
        break_ = true;
        return;
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
    auto current_pose = move_group_->getCurrentPose("suction_link");
    RCLCPP_INFO(get_logger(),
        "Goal pose: x=%f y=%f z=%f",
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z);
    if(poseEqual(current_pose, target)) return true;
    move_group_->setPoseTarget(target, end_effector);

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
        //if(blocking) visual_tools_->prompt("Execute planned trajectory?");
        return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }
    return false;
}

void ArmController::controlLoop()
{   
    if (break_)
    {
        disableVacuum();
        return;
    }
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
    RobotCommand cmd;
    auto current_pose = move_group_->getCurrentPose("suction_link");
    {
        std::lock_guard<std::mutex> lock(context_mutex_);
        cmd = state_machine_.update(context_);
    }
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
            RCLCPP_INFO(get_logger(), "Wait command");
            holdForUser();
            break;
        case CommandType::MoveArm:
            RCLCPP_INFO(get_logger(), "Move command");
            moveToPose(cmd.pose);
            break;
        case CommandType::SelectNextApple:
            RCLCPP_INFO(get_logger(), "Next Apple command");
            context_.target_pose = getNextPose();
            break;
        case CommandType::StartSuction:
            RCLCPP_INFO(get_logger(), "Start suction command");
            triggerVacuum();
            break;
        case CommandType::StopSuction:
            RCLCPP_INFO(get_logger(), "Stop suction here!");
            disableVacuum();
            break;           
        case CommandType::StopArm:
            RCLCPP_INFO(get_logger(), "Stop command");
            //move_group_->stop();
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
    {
        std::lock_guard<std::mutex> lock(context_mutex_);
        context_.at_pose =
            poseEqual(move_group_->getCurrentPose("suction_link"), cmd.pose);
    }
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
bool ArmController::holdForUser() {
    RCLCPP_INFO(get_logger(), "Execute next step?");
    block = true;
    while(rclcpp::ok() && block && !break_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return true;
}

void ArmController::triggerVacuum() {
    if (!suction_client_->wait_for_service(std::chrono::milliseconds(500))) {
        RCLCPP_ERROR(get_logger(), "Suction service not available");
        return;
    }
    RCLCPP_INFO(get_logger(), "Sending suction request: ON");
    auto req = std::make_shared<apple_interfaces::srv::SuctionCommand::Request>();
    req->relay_id = relay_num;
    req->state = true;
    suction_client_->async_send_request(req);
    suction_running_ = true;
    last_suction_ = std::chrono::steady_clock::now();
}
void ArmController::disableVacuum() {
    if (!suction_client_->wait_for_service(std::chrono::milliseconds(500))) {
        RCLCPP_ERROR(get_logger(), "Suction service not available");
        return;
    }
    RCLCPP_INFO(get_logger(), "Sending suction request: OFF");
    auto req = std::make_shared<apple_interfaces::srv::SuctionCommand::Request>();
    req->relay_id = relay_num;
    req->state = false;
    auto future = suction_client_->async_send_request(req);
}
void ArmController::monitorVacuum() {
    if(suction_running_) {
        if (std::chrono::steady_clock::now() - last_suction_ > suction_timeout_) {
            suction_running_ = false;
            disableVacuum();
            return;
        }

        if (context_.suction_state) {
            suction_running_ = false;
            return; // success
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    return;
}
geometry_msgs::msg::PoseStamped ArmController::getNextPose() {
    auto target = apple_pose_queue_.front();
    apple_pose_queue_.pop_front();
    {
        std::lock_guard<std::mutex> ctx_lock(context_mutex_);
        context_.consensus_size--;
    }
    return *target;
}