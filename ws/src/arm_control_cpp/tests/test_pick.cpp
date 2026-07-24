#include "arm_control_cpp/arm_control.hpp"

#include <atomic>
#include <csignal>
#include <memory>
#include <cassert>


class TestArmController : public ArmController
{
public:
    using ArmController::getNextPose;
    using ArmController::createAppleScan;
    using ArmController::context_;
    using ArmController::controlLoop;
    using ArmController::suction_timeout_;
};

std::atomic<bool> shutdown_requested{false};
std::shared_ptr<TestArmController> node;

void sigintHandler(int);

int main(int argc, char * argv[])
{
    rclcpp::InitOptions options;
    options.shutdown_on_signal = false;

    rclcpp::init(argc, argv);

    std::signal(SIGINT, sigintHandler);

    node = std::make_shared<TestArmController>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);

    auto spinner = std::thread([&executor]() {executor.spin(); });

    node->initializeMoveIt();

    RCLCPP_INFO(node->get_logger(), "Queue up apple pose for testing.");
    node->holdForUser();
    assert(node->createAppleScan() && "VISION SCAN FAILURE!");
    // Arbitrary sleep
    rclcpp::sleep_for(std::chrono::seconds(5));
    assert(node->context_.consensus_size >= 1 && "NO CONSENSUS FOUND!");
    auto pose = node->getNextPose();
    RCLCPP_INFO(
        node->get_logger(),
        "PoseStamped [frame=%s] Pos(%.3f, %.3f, %.3f) Orient(%.3f, %.3f, %.3f, %.3f)",
        pose.header.frame_id.c_str(),
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w);

    RCLCPP_WARN(node->get_logger(), "THIS STEP WILL MOVE THE ROBOT!");
    node->holdForUser();

    // Create seeded context
    node->context_.state = RobotState::Approach;
    node->context_.target_pose = pose;

    // Iterate state from seeded context
    node->controlLoop();

    // Iterate to verify pose
    node->controlLoop();

    assert(node->context_.at_pose && "FAILED TO REACH POSE!");

    RCLCPP_WARN(node->get_logger(), "VISUALLY INSPECT ROBOT: suction should be ~10mm from apple");

    node->holdForUser();

    // Iterate to start suction
    node->controlLoop();

    // Iterate to move to apple
    node->controlLoop();

    // Sleep to let apple settle
    rclcpp::sleep_for(node->suction_timeout_);
    rclcpp::sleep_for(std::chrono::seconds(1)); // Give suction time to stop
    assert(node->context_.suction_state && "Failed to grasp aple!");
    // Vacuum should disable from timeout

    // Disable vacuum but override suction state
    node->disableVacuum();
    node->context_.suction_state = true;
    node->context_.last_qr_scan = std::chrono::steady_clock::now(); // Override QR Behaviour

    RCLCPP_WARN(node->get_logger(), "VISUALLY INSPECT ROBOT: suction should be ~10mm from apple");
    RCLCPP_WARN(node->get_logger(), "CONTINUE TO WATCH ROBOT: robot will complete pick-twist action");
    node->holdForUser();
    
    while(node->context_.state != RobotState::ChutePrepare) {
        node->controlLoop();
    }

    node->context_.state = RobotState::Monitor;
    node->controlLoop();

    RCLCPP_INFO(node->get_logger(), "All tests succeeded!");
    rclcpp::sleep_for(std::chrono::milliseconds(500)); 

    executor.cancel();  
    spinner.join();

    rclcpp::shutdown();

    return 0;
}

void sigintHandler(int)
{
    shutdown_requested = true;

    RCLCPP_INFO(node->get_logger(), "Stopping vacuum...");

    auto future = node->disableVacuum();

    if (future.valid())
    {
        auto ret = rclcpp::spin_until_future_complete(
            node,
            future,
            std::chrono::seconds(2));

        if (ret != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_WARN(node->get_logger(),
                        "Timed out waiting for vacuum to disable.");
        }
    }

    rclcpp::shutdown();
}