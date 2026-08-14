#include "arm_control_cpp/arm_control.hpp"
#include "arm_control_cpp/robot_state_enum.hpp"
#include "arm_control_cpp/state_machine.hpp"
#include "arm_control_cpp/utils.hpp"
moveit_msgs::msg::RobotTrajectory concatenateTrajectories(
    const moveit_msgs::msg::RobotTrajectory &traj1,
    const moveit_msgs::msg::RobotTrajectory &traj2);
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
        std::chrono::milliseconds(200),
        std::bind(&ArmController::monitorVacuum, this),
        timer_callback_group_
    );
    // timer_ = create_wall_timer(
    //     std::chrono::milliseconds(100),
    //     std::bind(&ArmController::controlLoop, this),
    //     timer_callback_group_
    // );

    scan_pose_client_ = create_client<apple_interfaces::srv::ScanPoseRequest>("/scout1_cam/scan_pose_request");
    scan_pose_clear_pub_ = create_publisher<std_msgs::msg::UInt32>("/scout1_cam/clear_pose", 10);

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
    // createAppleScan();
    // while (context_.consensus_size == 0)
    // {
    //     rclcpp::sleep_for(std::chrono::milliseconds(500));
    // }
}
void ArmController::test_function()
{
    robot_trajectory::RobotTrajectory total_traj(move_group_->getRobotModel(), "arm_1");
    
    context_.state = RobotState::QRScan;
    auto pose = getPoseForState(context_);
    moveit::core::RobotState current_state = *move_group_->getCurrentState();
    auto plan = planMotion(current_state, pose, "suction_link");
    if(plan)
    {
        const auto &traj = (*plan).trajectory_.joint_trajectory;
        robot_trajectory::RobotTrajectory robot_traj(move_group_->getRobotModel(), "arm_1");
        robot_traj.setRobotTrajectoryMsg(current_state, (*plan).trajectory_);
        RCLCPP_INFO(
            get_logger(),
            "Trajectory: %zu points, %zu joints",
            traj.points.size(),
            traj.joint_names.size());
        total_traj.append(robot_traj, 0);
    } else 
    {
        RCLCPP_ERROR(get_logger(), "No plan given!");
    }

    context_.state = RobotState::Chute;
    auto chute_pose = getPoseForState(context_);
    const moveit::core::RobotState& qr_state = total_traj.getLastWayPoint();
    plan = planMotion(qr_state, chute_pose, "suction_link");
    if(plan)
    {
        const auto &traj = (*plan).trajectory_.joint_trajectory;
        robot_trajectory::RobotTrajectory robot_traj(move_group_->getRobotModel(), "arm_1");
        robot_traj.setRobotTrajectoryMsg(qr_state, (*plan).trajectory_);
        RCLCPP_INFO(
            get_logger(),
            "Trajectory: %zu points, %zu joints",
            traj.points.size(),
            traj.joint_names.size());
        total_traj.append(robot_traj, 0);
    } else 
    {
        RCLCPP_ERROR(get_logger(), "No plan given!");
    }

    auto home_pose = move_group_->getCurrentPose("suction_link");
    const moveit::core::RobotState& new_state = total_traj.getLastWayPoint();
    plan = planMotion(new_state, home_pose, "suction_link");
    if(plan)
    {
        const auto &traj = (*plan).trajectory_.joint_trajectory;
        robot_trajectory::RobotTrajectory robot_traj(move_group_->getRobotModel(), "arm_1");
        robot_traj.setRobotTrajectoryMsg(new_state, (*plan).trajectory_);
        RCLCPP_INFO(
            get_logger(),
            "Trajectory: %zu points, %zu joints",
            traj.points.size(),
            traj.joint_names.size());
        total_traj.append(robot_traj, 0);
    } else 
    {
        RCLCPP_ERROR(get_logger(), "No plan given!");
    }

    moveit_msgs::msg::RobotTrajectory new_traj_msg;
    total_traj.getRobotTrajectoryMsg(new_traj_msg);
    // Execute the modified trajectory
    move_group_->execute(new_traj_msg);
    throw std::runtime_error("testing");
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
void ArmController::logPose(const std::string poseContext, const geometry_msgs::msg::PoseStamped& pose)
{
    RCLCPP_INFO(
        get_logger(),
        "%s [frame=%s] Pos(%.3f, %.3f, %.3f) Orient(%.3f, %.3f, %.3f, %.3f)",
        poseContext.c_str(),
        pose.header.frame_id.c_str(),
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w);
}
void ArmController::suctionCallback(
    const std_msgs::msg::UInt8::SharedPtr msg)
{
    latest_vacuum_state_ = msg->data;
    uint8_t mask = (16 << relay_num); // Last 4 bits of integer
    if (latest_vacuum_state_ & mask) vacuum_consensus_count_++;
    else vacuum_consensus_count_ = 0;
    if (vacuum_consensus_count_ > 25 && !context_.suction_state)
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
std::vector<double> generateBoundingBox(
    const geometry_msgs::msg::Pose& current_pose, 
    const geometry_msgs::msg::Pose& target_pose
)
{
    double buffer = 0.75; // meters

    double min_x = std::min(
        current_pose.position.x,
        target_pose.position.x
    ) - buffer;

    double max_x = std::max(
        current_pose.position.x,
        target_pose.position.x
    ) + buffer;


    double min_y = std::min(
        current_pose.position.y,
        target_pose.position.y
    ) - buffer;

    double max_y = std::max(
        current_pose.position.y,
        target_pose.position.y
    ) + buffer;


    double min_z = std::min(
        current_pose.position.z,
        target_pose.position.z
    ) - buffer;

    double max_z = std::max(
        current_pose.position.z,
        target_pose.position.z
    ) + buffer;
    
    std::vector<double> bbox {
        min_x,
        min_y,
        min_z,
        max_x,
        max_y,
        max_z
    };

    return bbox;
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

    auto current_pose = move_group_->getCurrentPose(end_effector);

    logPose("Move Goal Pose: ", target);
    if(poseEqual(current_pose, target)) return true;
    move_group_->clearPoseTargets();
    move_group_->setStartStateToCurrentState();
    if (!move_group_->setPoseTarget(target, end_effector))
    {
        RCLCPP_ERROR(get_logger(), "Failed to set pose target.");
        return false;
    }
    std::vector<double> bbox = generateBoundingBox(current_pose.pose, target.pose);

    move_group_->setWorkspace(
        bbox[0], bbox[1], bbox[2], bbox[3], bbox[4], bbox[5]
    );

    bool planned;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    for(int i = 0; i < 3; i++)
    {
        move_group_->setPlanningPipelineId("ompl");
        move_group_->setPlannerId("RRTstar");
        move_group_->setPlanningTime(0.7);
        move_group_->setNumPlanningAttempts(3);

        planned = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!planned)
        {
            RCLCPP_WARN(get_logger(), "Planning failed.");
        }
        else
        {
            break;
        }
    }
    if(!planned)
    {
        RCLCPP_ERROR(get_logger(), "Failed to plan after multiple attempts!");
        return false;
    }

    const auto &traj = plan.trajectory_.joint_trajectory;

    RCLCPP_INFO(
        get_logger(),
        "Trajectory: %zu points, %zu joints",
        traj.points.size(),
        traj.joint_names.size());

    // for (size_t i = 0; i < traj.points.size(); ++i)
    // {
    //     const auto &p = traj.points[i];

    //     RCLCPP_INFO(
    //         get_logger(),
    //         "Point %zu: t=%.3f s",
    //         i,
    //         p.time_from_start.sec +
    //             p.time_from_start.nanosec * 1e-9);

    //     for (size_t j = 0; j < p.positions.size(); ++j)
    //     {
    //         RCLCPP_INFO(
    //             get_logger(),
    //             "  %s: pos=%.4f vel=%.4f",
    //             traj.joint_names[j].c_str(),
    //             p.positions[j],
    //             j < p.velocities.size() ? p.velocities[j] : 0.0);
    //     }
    // }

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
std::optional<moveit::planning_interface::MoveGroupInterface::Plan> ArmController::planMotion(
    const moveit::core::RobotState& start_state,
    const geometry_msgs::msg::PoseStamped& target_pose,
    const std::string& end_effector
)
{
    try
    {
        moveit::core::RobotState state = start_state;

        const auto* joint_model_group = move_group_->getRobotModel()->getJointModelGroup("arm_1");
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

        std::vector<double> joint_values;
        start_state.copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
        RCLCPP_INFO(get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }

        // Make sure all link transforms have been recomputed
        state.update();

        const auto* link_model =
            state.getRobotModel()->getLinkModel(end_effector);

        if (!link_model)
        {
            RCLCPP_ERROR(
                get_logger(),
                "Link '%s' does not exist in robot model",
                end_effector.c_str());

            return std::nullopt;
        }

        auto start_tf = state.getGlobalLinkTransform(end_effector).translation();
        if (!start_tf.allFinite())
        {
            RCLCPP_ERROR(
                get_logger(),
                "getGlobalLinkTransform() returned a non-finite transform for link '%s'",
                end_effector.c_str());
            return std::nullopt;
        }
        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.pose.position.x = start_tf[0];
        start_pose.pose.position.y = start_tf[1];
        start_pose.pose.position.z = start_tf[2];
        start_pose.header.frame_id = "base_link";
        logPose("Start pose for plan: ", start_pose);
        auto bbox = generateBoundingBox(start_pose.pose, target_pose.pose);
        move_group_->clearPoseTargets();
        move_group_->setStartState(state);
        if(!move_group_->setPoseTarget(target_pose, end_effector))
        {
            RCLCPP_ERROR(get_logger(), "Failed to set target pose!");
            return std::nullopt;
        }
        move_group_->setWorkspace(
            bbox[0], bbox[1], bbox[2], bbox[3], bbox[4], bbox[5]
        );

        bool planned = false;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        for(int i = 0; i < 3; i++)
        {
            move_group_->setPlanningPipelineId("ompl");
            move_group_->setPlannerId("RRTstar");
            move_group_->setPlanningTime(0.7);
            move_group_->setNumPlanningAttempts(3);

            auto result = move_group_->plan(plan);

            planned = (result == moveit::core::MoveItErrorCode::SUCCESS);

            if (!planned)
            {
                RCLCPP_WARN(get_logger(), "Planning failed.");
                RCLCPP_ERROR(
                    get_logger(),
                    "MoveIt plan() returned: %d",
                    result.val);
            }
            else
            {
                break;
            }
        }

        if(!planned)
        {
            RCLCPP_ERROR(get_logger(), "Failed to plan after multiple attempts!");
            return std::nullopt;
        }

        return plan;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(get_logger(), "Planning threw an exception: %s", e.what());
        return std::nullopt;
    }
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
    logPose("Command Pose: ", pose);
    logPose("Current Pose: ", move_group_->getCurrentPose("suction_link"));


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
        case CommandType::GetNextScanPose:
            RCLCPP_INFO(get_logger(), "Get Next Scan Pose Command");
            getScanPose();
            break;
        default:
            RCLCPP_INFO(get_logger(), "No command specified!");
            rclcpp::sleep_for(std::chrono::milliseconds(500));
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
        rclcpp::sleep_for(std::chrono::milliseconds(500));
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
    rclcpp::sleep_for(std::chrono::milliseconds(300));
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
    // rclcpp::sleep_for(std::chrono::milliseconds(400));
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
        context_.vision_scan_available = false;
    }  
    VisionScan::Goal goal_msg;
    goal_msg.order = scan_type;
    goal_msg.qr_message = "BIN_FLAG_SW";
    goal_msg.stamp = get_clock()->now();

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
        logPose("QR Pose: ", pose);
        updateBinPose(pose);
    }
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
            break;
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
    // Any case resets the flag
    {
        std::lock_guard<std::mutex> ctx_lock(context_mutex_);
        context_.vision_scan_available = true;
    }
    auto msg = std_msgs::msg::UInt32();
    msg.data = last_scan_ID;
    scan_pose_clear_pub_->publish(msg);
    last_scan_ID = 0;
}

geometry_msgs::msg::PoseStamped ArmController::getNextPose() {
    if (apple_pose_queue_.size() == 0)
    {
        throw std::runtime_error("No apple pose available!");
    }
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
            context_.vision_scan_available = true;
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

void ArmController::getScanPose()
{
    {
        std::lock_guard<std::mutex> ctx_lock(context_mutex_);
        context_.vision_scan_available = false;
    }
    using ServiceResponseFuture =
        rclcpp::Client<apple_interfaces::srv::ScanPoseRequest>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        auto result = future.get();
            if(result->success)
            {   
                geometry_msgs::msg::PoseStamped pose;

                try
                {
                    tf_buffer_->transform(
                        result->pose,
                        pose,
                        "base_link",
                        std::chrono::seconds(2)
                    );
                }
                catch (const tf2::TransformException& ex)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Failed to transform pose: %s",
                        ex.what()
                    );
                    std::lock_guard<std::mutex> ctx_lock(context_mutex_);
                    context_.general_command_fail = true;
                    context_.vision_scan_available = true;
                    return;
                }
                std::lock_guard<std::mutex> ctx_lock(context_mutex_);
                context_.target_pose = pose;
                context_.target_pose.pose.orientation.x = -0.5;
                context_.target_pose.pose.orientation.y = 0.5;
                context_.target_pose.pose.orientation.z = -0.5;
                context_.target_pose.pose.orientation.w = 0.5;
                context_.vision_scan_available = true;
                last_scan_ID = result->id;
                RCLCPP_INFO(get_logger(), "Vision Scan Pose Gotten");
            }
            else
            {
                std::lock_guard<std::mutex> ctx_lock(context_mutex_);
                context_.general_command_fail = true;
                context_.vision_scan_available = true;
            }
        };
    auto req = std::make_shared<apple_interfaces::srv::ScanPoseRequest::Request>();
    req->arm_num = 1;
    auto result = scan_pose_client_->async_send_request(req, response_received_callback);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
}