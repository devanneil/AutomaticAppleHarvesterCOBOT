#include "arm_control_cpp/arm_control.hpp"

#include <atomic>
#include <csignal>
#include <memory>
#include <cassert>

class TestArmController : public ArmController
{
public:
    using ArmController::getNextPose;
    using ArmController::createQRScan;
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

    // Create seeded context
    node->context_.state = RobotState::CloseScan;

    while (node->context_.state != RobotState::Monitor)
    {
        node->controlLoop();
    }
    node->controlLoop();

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