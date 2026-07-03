#include "arm_control_cpp/arm_control.hpp"

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
        node->controlLoop();
        if (node->break_)
        {   
            node->disableVacuum();
            break;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(100));
    }
    executor.cancel();  
    spinner.join();

    rclcpp::shutdown();

    return 0;
}