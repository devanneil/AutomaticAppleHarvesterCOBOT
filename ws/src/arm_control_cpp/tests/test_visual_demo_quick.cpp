#include "arm_control_cpp/arm_control.hpp"

#include <atomic>
#include <csignal>
#include <memory>
#include <cassert>
#include <cstdlib>


class TestArmController : public ArmController
{
public:
    using ArmController::moveToPose;
    using ArmController::cartesianMove;
};

std::atomic<bool> shutdown_requested{false};
std::shared_ptr<TestArmController> node;

void sigintHandler(int);
bool qr_ready_check();

geometry_msgs::msg::PoseStamped demo_twist_pick(geometry_msgs::msg::PoseStamped pose, double twist_angle)
{
    double twist_rad = degreesToRadians(twist_angle);
    return create_pose(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, -M_PI/2 - twist_rad, 0, 0, "base_link");
}

int rand_steps = 100;
float x_max_dist = 0.25;
float z_max_dist = 0.25;
float roll_max_deg = 60;
geometry_msgs::msg::PoseStamped randomize_pose(geometry_msgs::msg::PoseStamped pose)
{
    float x_scale = 2.0 * (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) - 0.5f);
    float z_scale = 2.0 * (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) - 0.5f);
    std::cout << x_scale << " " << z_scale << std::endl;
    // x_scale = 1.0;
    // z_scale = 1.0;
    float degrees_scale = 2.0 * (static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX) - 0.5f);
    float x_val = pose.pose.position.x + x_max_dist * x_scale;
    float z_val = pose.pose.position.z + z_max_dist * z_scale;
    float rad_val = degreesToRadians(roll_max_deg * degrees_scale);
    return create_pose(x_val, 1.2, z_val, -M_PI/2, rad_val, 0.0, "base_link");
}

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
    std::srand(node->get_clock()->now().seconds());

    node->initializeMoveIt();

    // Do work here
    auto monitor_pose = create_pose(0.161, 0.45, 1.390, -M_PI/2, 0.0, 0.0, "base_link");
    node->moveToPose(monitor_pose, false, "suction_link");

    double chute_twist_degrees = 30;
    double twist_rad = degreesToRadians(chute_twist_degrees);
    auto chute_pose = create_pose(-0.4, 0.1, 1.0, M_PI - twist_rad, 0.0, 0.0, "base_link");
    //node->moveToPose(chute_pose, false, "link_6");

    while(true)
    {
        std::cout << "\n====================================\n";
        std::cout << " Demo is ready.\n";
        std::cout << " Press ENTER to begin...\n";
        std::cout << " Press ctrl-c to exit... (you may need to press multiple times)\n";
        std::cout << "====================================\n";

        std::cin.get();

        for(int i = 0; i < 4; i++)
        {
            auto center_pose = create_pose(0.161, 1.2, 1.390, -M_PI/2, 0.0, 0.0, "base_link");
            auto rand_pose = randomize_pose(center_pose);
            node->moveToPose(rand_pose, false, "suction_link");
            auto twist_pose = demo_twist_pick(rand_pose, 20.0);
            //node->moveToPose(twist_pose, false, "suction_link");
            auto retreat_pose = create_pose(rand_pose.pose.position.x, 0.9, rand_pose.pose.position.z, -M_PI/2, 0.0, 0.0, "base_link");
            //node->moveToPose(retreat_pose, false, "suction_link");

            node->cartesianMove({twist_pose, retreat_pose, chute_pose}, "suction_link");
        }

        node->moveToPose(monitor_pose, false, "suction_link");
    }


    node->moveToPose(monitor_pose, false, "suction_link");


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