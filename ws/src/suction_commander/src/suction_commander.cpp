#include <memory>
#include <chrono>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include "apple_interfaces/srv/suction_command.hpp"

std::string DEV = "/dev/ttyUSB0";

class SerialPort {
public:
    bool openPort(const std::string &device, int baud) {
        fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);

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
        tty.c_cc[VTIME] = 1;   // faster polling

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
        std::lock_guard<std::mutex> lock(io_mutex);

        uint8_t packet[2] = {header, data};
        ::write(fd, packet, 2);
    }

    int readPacket(uint8_t &header, uint8_t &data) {
        std::lock_guard<std::mutex> lock(io_mutex);

        uint8_t buf[2];
        int n = ::read(fd, buf, 2);

        if (n == 2) {
            header = buf[0];
            data = buf[1];
        }

        return n;
    }

    int getFd() const { return fd; }

    ~SerialPort() {
        if (fd >= 0) close(fd);
    }

private:
    int fd = -1;
    std::mutex io_mutex;
};

class SuctionCommander : public rclcpp::Node
{
public:
    SuctionCommander() : Node("suction_commander")
    {
        serial.openPort("/dev/ttyUSB0", 115200);

        cmd_sub = create_subscription<std_msgs::msg::UInt8>(
            "/suction/command", 10,
            std::bind(&SuctionCommander::cmdCallback, this, std::placeholders::_1));

        state_pub = create_publisher<std_msgs::msg::UInt8>(
            "/suction/state", 10);

        reader_thread = std::thread(&SuctionCommander::readLoop, this);
    }
    ~SuctionCommander() {
        serial.writePacket(0xAA, 0);
    }
private:

    void cmdCallback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        serial.writePacket(0xAA, msg->data);
    }

    void readLoop()
    {
        while (rclcpp::ok()) {

            uint8_t header = 0;
            uint8_t state = 0;

            if (::read(serial.getFd(), &header, 1) <= 0)
                continue;

            if (header != 0xAA)
                continue;

            if (::read(serial.getFd(), &state, 1) <= 0)
                continue;

            auto msg = std_msgs::msg::UInt8();
            msg.data = state;
            state_pub->publish(msg);
        }
    }

    SerialPort serial;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cmd_sub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub;

    std::thread reader_thread;
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