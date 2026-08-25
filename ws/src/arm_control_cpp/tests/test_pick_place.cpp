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
bool qr_ready_check();

int main(int argc, char * argv[])
{
    rclcpp::InitOptions options;
    options.shutdown_on_signal = false;

    rclcpp::init(argc, argv);

    std::signal(SIGINT, sigintHandler);

    assert(qr_ready_check() && "Failed to pass QR ready check! Scan the QR Pose again!" );

    node = std::make_shared<TestArmController>();

    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(node);

    auto spinner = std::thread([&executor]() {executor.spin(); });

    node->initializeMoveIt();

    // Create seeded context
    node->context_.state = RobotState::CloseScan;

    std::cout << "\n====================================\n";
    std::cout << " Demo is ready.\n";
    std::cout << " Press ENTER to begin...\n";
    std::cout << "====================================\n";

    std::cin.get();

    node->context_.last_state = std::chrono::steady_clock::now();

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

bool qr_ready_check()
{
    if (!rclcpp::ok())
        return false;

    auto qr_node =
        std::make_shared<rclcpp::Node>("qr_check_node");

    std::atomic<bool> received{false};
    std::atomic<bool> valid{false};

    auto qr_valid_sub =
        qr_node->create_subscription<std_msgs::msg::Bool>(
            "/qr_valid",
            rclcpp::QoS(1)
                .reliable()
                .transient_local(),
            [&received, &valid](
                const std_msgs::msg::Bool::SharedPtr msg)
            {
                valid.store(msg->data);
                received.store(true);
            });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(qr_node);

    const auto timeout =
        std::chrono::steady_clock::now() +
        std::chrono::seconds(2);

    while (rclcpp::ok() &&
           !received.load() &&
           std::chrono::steady_clock::now() < timeout)
    {
        executor.spin_some();
        std::this_thread::sleep_for(
            std::chrono::milliseconds(10));
    }

    executor.remove_node(qr_node);

    if (!received.load())
    {
        RCLCPP_ERROR(
            qr_node->get_logger(),
            "QR check failed: no /qr_valid state received.");

        return false;
    }

    if (valid.load())
    {
        RCLCPP_INFO(
            qr_node->get_logger(),
            "Passed the QR check!");

        return true;
    }

    RCLCPP_ERROR(
        qr_node->get_logger(),
        "Failed the QR check: QR state is invalid.");

    return false;
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