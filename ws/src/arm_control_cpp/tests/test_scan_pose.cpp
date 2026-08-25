#include "arm_control_cpp/arm_control.hpp"

#include <atomic>
#include <csignal>
#include <memory>
#include <cassert>
#include <iostream>
#include <cctype>

bool getYesNo()
{
    char input;

    while (true)
    {
        std::cout << "Enter y or n: ";
        std::cin >> input;

        input = std::tolower(static_cast<unsigned char>(input));

        if (input == 'y')
            return true;

        if (input == 'n')
            return false;

        std::cout << "Invalid input. Please enter y or n.\n";
    }
}

class TestArmController : public ArmController
{
public:
    using ArmController::getScanPose;
    using ArmController::clearScanPose;
    using ArmController::moveToPose;
    using ArmController::context_;
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

    // This function is bad
    node->getScanPose();
    auto scan_pose = node->context_.target_pose;
    assert((!node->context_.general_command_fail) && "NO SCAN POSE GIVEN!");

    auto result = node->moveToPose(scan_pose, false, "suction_link");
    assert(result && "FAILED TO MOVE TO POSE!");

    std::cout << "Enter yes to clear out this scan pose: " << std::endl;
    bool conf = getYesNo();
    if(conf)
    {
        node->clearScanPose();
    }

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