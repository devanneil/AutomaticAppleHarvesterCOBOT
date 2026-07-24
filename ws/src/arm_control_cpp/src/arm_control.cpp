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
    // apple_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    //     apple_pose_topic,
    //     10,
    //     std::bind( &ArmController::appleCallback, this, std::placeholders::_1),
    //     sub_options
    // );
    camera_client_ = rclcpp_action::create_client<apple_interfaces::action::VisionScan>(this, "/cam_drive_control/vision_scan");
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
    bin_manager_client_ = create_client<apple_interfaces::srv::UpdateBin>("/update_bin");
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
    context_.last_qr_scan = std::chrono::steady_clock::time_point::max();
    context_.suction_state = false;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}
ArmController::~ArmController() {
    stopArm();
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

    // createQRScan();
    // auto start = std::chrono::steady_clock::now();
    // while(!timeout_elapsed(start, std::chrono::seconds(5)))
    // {
    //     rclcpp::sleep_for(std::chrono::milliseconds(100));
    // }
    // auto pose = create_pose(0.01, 0.01, 0.05, -2.0, 0.0, 0, "Chute_SE");
    // moveToPose(pose, true, "suction_link");
    // throw std::runtime_error("testing");
}

// void ArmController::appleCallback(
//     const geometry_msgs::msg::PoseStamped::SharedPtr msg)
// {
//     if (!isDuplicatePose(msg))
//     {
//         msg->pose.position.z += 0.02;
//         msg->pose.position.y -= 0.02;
//         apple_pose_queue_.push_back(msg);
//     }
//     {
//         std::lock_guard<std::mutex> ctx_lock(context_mutex_);
//         context_.consensus_size++;
//     }
// }
void ArmController::suctionCallback(
    const std_msgs::msg::UInt8::SharedPtr msg)
{
    latest_vacuum_state_ = msg->data;
    uint8_t mask = (16 << relay_num); // Last 4 bits of integer
    if (latest_vacuum_state_ & mask) vacuum_consensus_count_++;
    else vacuum_consensus_count_ = 0;
    if (vacuum_consensus_count_ > 5 && !context_.suction_state)
    {
        std::lock_guard<std::mutex> lock(context_mutex_);
        context_.suction_state = true;
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
    bool blocking = false,
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
    move_group_->clearPoseTargets();
    move_group_->setStartStateToCurrentState();
    if (!move_group_->setPoseTarget(target, end_effector))
    {
        RCLCPP_ERROR(get_logger(), "Failed to set pose target.");
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool planned =
        (move_group_->plan(plan) ==
        moveit::core::MoveItErrorCode::SUCCESS);

    if (!planned)
    {
        RCLCPP_WARN(get_logger(), "Planning failed.");

        return false;
    }

    if (blocking)
        visual_tools_->prompt("Execute planned trajectory?");

    auto result = move_group_->execute(plan);

    RCLCPP_INFO(get_logger(),
            "Execute returned %d",
            result.val);


    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Trajectory execution failed.");

        stopArm();

        //throw std::runtime_error("Arm control execute failure!");
        return false;
    }

    return true;
}
void ArmController::stopArm() {
    move_group_->stop();

    rclcpp::sleep_for(std::chrono::seconds(5));

    move_group_->clearPoseTargets();
    move_group_->setStartStateToCurrentState();
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
            context_.move_command_fail = !moveToPose(cmd.pose);
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
            stopArm();
            break;
        case CommandType::VisionScan:
            RCLCPP_INFO(get_logger(), "Vision scan command");
            createAppleScan();
            break;
        case CommandType::QRScan:
            RCLCPP_INFO(get_logger(), "QR Scan command");
            createQRScan();
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
    // {
    //     std::lock_guard<std::mutex> lock(context_mutex_);
    //     context_.suction_state = true;
    // }  
    suction_running_ = true;
    last_suction_ = std::chrono::steady_clock::now();
}
rclcpp::Client<apple_interfaces::srv::SuctionCommand>::SharedFuture ArmController::disableVacuum() {
    if (!suction_client_->wait_for_service(std::chrono::milliseconds(500))) {
        RCLCPP_ERROR(get_logger(), "Suction service not available");
        return {};
    }
    RCLCPP_INFO(get_logger(), "Sending suction request: OFF");
    auto req = std::make_shared<apple_interfaces::srv::SuctionCommand::Request>();
    req->relay_id = relay_num;
    req->state = false;
    auto future = suction_client_->async_send_request(req);
    {
        std::lock_guard<std::mutex> ctx_lock(context_mutex_);
        context_.suction_state = false;
    }
    suction_running_ = false;
    return future;
}
void ArmController::monitorVacuum() {
    if(suction_running_) {
        if (timeout_elapsed(last_suction_, suction_timeout_)) {
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
bool ArmController::createVisionScan(uint8_t scan_type)
{
    if (!camera_client_->wait_for_action_server(
            std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Vision scan action server unavailable"
        );
        return false;
    }
    {
        std::lock_guard<std::mutex> ctx_lock(context_mutex_);
        context_.vision_scan_fail = false;
    }  
    VisionScan::Goal goal_msg;
    goal_msg.order = scan_type;

    rclcpp_action::Client<VisionScan>::SendGoalOptions options;

    options.goal_response_callback =
        std::bind(
            &ArmController::goal_response_callback,
            this,
            std::placeholders::_1
        );

    options.feedback_callback =
        std::bind(
            &ArmController::feedback_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        );

    options.result_callback =
        std::bind(
            &ArmController::result_callback,
            this,
            std::placeholders::_1
        );

    camera_client_->async_send_goal(goal_msg, options);
    last_goal_order_ = scan_type;

    return true;
}

bool ArmController::createAppleScan()
{
    return createVisionScan(
        VisionScan::Goal::APPLE_SCAN
    );
}


bool ArmController::createQRScan()
{
    return createVisionScan(
        VisionScan::Goal::QR_SCAN
    );
}

void ArmController::goal_response_callback(
    GoalHandleVisionScan::SharedPtr goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Vision scan goal rejected"
        );
        return;
    }

    vision_goal_handle_ = goal_handle;

    RCLCPP_INFO(
        this->get_logger(),
        "Vision scan goal accepted"
    );
}

void ArmController::feedback_callback(
    GoalHandleVisionScan::SharedPtr,
    const std::shared_ptr<const VisionScan::Feedback> feedback)
{
    RCLCPP_INFO(
        this->get_logger(),
        "Vision feedback success: %s",
        feedback->success ? "true" : "false"
    );
    if (last_goal_order_ == VisionScan::Goal::APPLE_SCAN)
    {
        for (const auto & apple : feedback->apples)
        {
            apple_pose_queue_.push_back(std::make_shared<geometry_msgs::msg::PoseStamped>(apple));
            {
                std::lock_guard<std::mutex> ctx_lock(context_mutex_);
                context_.consensus_size++;
            }
        }
    }
    if (last_goal_order_ == VisionScan::Goal::QR_SCAN)
    {
        auto pose = feedback->qr_pose;
        RCLCPP_INFO(
            get_logger(),
            "QRPose [frame=%s] Pos(%.3f, %.3f, %.3f) Orient(%.3f, %.3f, %.3f, %.3f)",
            pose.header.frame_id.c_str(),
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w);
        updateBinPose(pose);
    }
    vision_goal_handle_.reset();
}

void ArmController::result_callback(
    const GoalHandleVisionScan::WrappedResult & result)
{
    switch(result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            RCLCPP_INFO(
                this->get_logger(),
                "Vision scan succeeded"
            );

            auto result_msg = result.result;

            // Example:
            // result_msg->apples
            // result_msg->qr_pose

            return;
        }

        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(
                this->get_logger(),
                "Vision scan aborted"
            );
            break;

        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(
                this->get_logger(),
                "Vision scan canceled"
            );
            break;

        default:
            RCLCPP_ERROR(
                this->get_logger(),
                "Unknown result code"
            );
            break;
    }
    // Any case apart from SUCCESS is fail
    {
        std::lock_guard<std::mutex> ctx_lock(context_mutex_);
        context_.vision_scan_fail = true;
    }
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

void ArmController::updateBinPose(geometry_msgs::msg::PoseStamped qr_pose)
{
    geometry_msgs::msg::TransformStamped t_dummy_qr;
    try {
        t_dummy_qr = tf_buffer_->lookupTransform(
            "dummy_link", "QR_SE",
            tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform dummy_link to QR_SE: %s", ex.what());
        return;
    }
    float x_dummy_qr = t_dummy_qr.transform.translation.x;
    float y_dummy_qr = t_dummy_qr.transform.translation.y;
    float z_dummy_qr = t_dummy_qr.transform.translation.z;

    float x_qr_base = qr_pose.pose.position.x;
    float y_qr_base = qr_pose.pose.position.y;
    float z_qr_base = qr_pose.pose.position.z;

    float x_dummy_base = x_qr_base - x_dummy_qr;
    float y_dummy_base = y_qr_base - y_dummy_qr;
    float z_dummy_base = z_qr_base - z_dummy_qr;

    if(
        x_dummy_base > 0
        || y_dummy_base < 0
        || (z_dummy_base < -0.1 || z_dummy_base > 0.1)
    ) 
    {
        RCLCPP_ERROR(
            this->get_logger(), "INVALID QR MOVE: %f %f %f", x_dummy_base, y_dummy_base, z_dummy_base
        );
        //Reject pose
        {
            std::lock_guard<std::mutex> ctx_lock(context_mutex_);
            context_.vision_scan_fail = true;
        }     
        return;
    }

    auto req = std::make_shared<apple_interfaces::srv::UpdateBin::Request>();
    req->new_pose[0] = x_dummy_base;
    req->new_pose[1] = y_dummy_base;
    req->new_pose[2] = z_dummy_base;
    using ServiceResponseFuture =
        rclcpp::Client<apple_interfaces::srv::UpdateBin>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
            if(result->success)
            {
                std::lock_guard<std::mutex> ctx_lock(context_mutex_);
                context_.last_qr_scan = std::chrono::steady_clock::now();
            }
        };
    auto result = bin_manager_client_->async_send_request(req, response_received_callback);
}