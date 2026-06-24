#include <memory>
#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include "apple_interfaces/srv/suction_command.hpp"

std::string DEV = "/dev/ttyUSB0";

class SerialPort {
public:
    bool openPort(const std::string &device, int baud) {
        fd = open(device.c_str(), O_RDWR | O_NOCTTY);

        if (fd < 0) {
            perror("Failed to open serial port");
            return false;
        }

        struct termios tty;
        memset(&tty, 0, sizeof tty);

        if (tcgetattr(fd, &tty) != 0) {
            perror("tcgetattr");
            return false;
        }

        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;

        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            perror("tcsetattr");
            return false;
        }

        return true;
    }

    void writePacket(uint8_t header, uint8_t data) {
        uint8_t packet[2] = {header, data};
        write(fd, packet, 2);
    }

    ~SerialPort() {
        if (fd >= 0) close(fd);
    }

private:
    int fd = -1;
};
class SuctionCommander : public rclcpp::Node
{
public:
    SuctionCommander() : Node("suction_Commander") {

        serial.openPort(DEV, 115200);

        _service = this->create_service<apple_interfaces::srv::SuctionCommand>(
            "suction_action",
            std::bind(&SuctionCommander::service_callback, this,
                std::placeholders::_1, std::placeholders::_2));
    }
    ~SuctionCommander() {
        serial.writePacket(0xAA, 0b0000); //Stop all relays
    }

private:

    void service_callback(
        const std::shared_ptr<apple_interfaces::srv::SuctionCommand::Request> request,
        std::shared_ptr<apple_interfaces::srv::SuctionCommand::Response> response)
    {
        if (!request) {
            RCLCPP_ERROR(this->get_logger(), "Null request");
            response->success = false;
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Relay request: %d", request->relay_id);
        
        if(request->state) {
            // Enable Relay
            relayMask |= (1 << request->relay_id);
        } else {
            relayMask &= ~(1 << request->relay_id);
        }


        // ---- SEND PACKET ----
        serial.writePacket(0xAA, relayMask);

        RCLCPP_INFO(this->get_logger(),
                    "Sent mask: 0x%02X", relayMask);

        response->success = true;
    }

    rclcpp::Service<apple_interfaces::srv::SuctionCommand>::SharedPtr _service;

    uint8_t relayMask = 0;

    SerialPort serial;
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