#include <memory>
#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "suction_interfaces/srv/suction_command.hpp"

std::string DEV = "/dev/ttyUSB0";

class SuctionCommander : public rclcpp::Node
{
public:
    SuctionCommander() : Node("suction_Commander") {
        _service = this->create_service<suction_interfaces::srv::SuctionCommand>(
            "suction_action", 
            std::bind(&SuctionCommander::service_callback, this,
                std::placeholders::_1, std::placeholders::_2));
    }
    ~SuctionCommander() {
        
    }
private:
    void service_callback(        
        const std::shared_ptr<suction_interfaces::srv::SuctionCommand::Request> request,
        std::shared_ptr<suction_interfaces::srv::SuctionCommand::Response> response) {
            // Input validation (optional)
            if (!request) {
                RCLCPP_ERROR(this->get_logger(), "Received null request pointer.");
                return;
            }

            RCLCPP_INFO(this->get_logger(),
                        "Incoming request: %d", request->relay_id);
            response->success = true;
        };
    
    rclcpp::Service<suction_interfaces::srv::SuctionCommand>::SharedPtr _service;

    uint8_t relayMask = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<SuctionCommander>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}