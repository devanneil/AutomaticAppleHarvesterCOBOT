#include "arm_control_cpp/arm_control.hpp"

#include <atomic>
#include <csignal>
#include <memory>
#include <cassert>

std::atomic<bool> shutdown_requested{false};
std::shared_ptr<TestArmController> node;

class TestArmController : public ArmController
{
public:
    using ArmController::suction_running_;
    using ArmController::suction_timeout_;
    using ArmController::triggerVacuum;
    using ArmController::disableVacuum;
};

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

    // Arbitrary sleep for service initialization
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Trigger suction
    node->triggerVacuum();
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    assert(node->suction_running_ && "NODE SUCTION TRIGGER FAILURE!");
    // Disable suction
    node->disableVacuum();
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    assert(!node->suction_running_ && "NODE SUCTION DISABLE FAILURE!");
    // Test timeout
    node->triggerVacuum();
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    assert(node->suction_running_ && "NODE SUCTION TIMEOUT TRIGGER FAILURE!");

    rclcpp::sleep_for(node->suction_timeout_);
    assert(!node->suction_running_ && "NODE SUCTION TIMEOUT DISABLE FAILURE!");
    
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