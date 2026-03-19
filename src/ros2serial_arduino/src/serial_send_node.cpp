#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

using std::placeholders::_1;

int fd1;

class MySubscriber : public rclcpp::Node
{
public:
    MySubscriber()
        : Node("my_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MySubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
        char buf[64]; // メッセージのバッファーサイズを適切に制限
        unsigned int bytes_written;

        RCLCPP_INFO(this->get_logger(), "I heard: '%lf'", msg->linear.x);

        // メッセージをバッファに書き込む
        // bytes_written = snprintf(buf, sizeof(buf), "%7.5f,%7.5f\n", msg->linear.x, msg->angular.z);
        bytes_written = snprintf(buf, sizeof(buf), "%7.3f,%7.3f,%7.3f,%7.3f\n", msg->linear.x, msg->angular.z, msg->angular.x, msg->angular.y);


        if (bytes_written > sizeof(buf)) {
            RCLCPP_ERROR(this->get_logger(), "Serial Fail: message formatting error");
            return;
        }
        else
        {
            printf("cmd_vel recv:%s\n", buf);

            int rec = write(fd1, buf, bytes_written);

            if (rec >= 0) {
                printf("Serial send:%s\n", buf);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Serial Fail: could not write");
            }
        }


    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int open_serial(const char *device_name)
{
    int fd = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    fcntl(fd, F_SETFL, 0);

    if (fd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Serial Fail: could not open %s", device_name);
        return -1;
    }

    // 以下略...
    return fd;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("Serialport");

    char device_name[] = "/dev/arduino-mega";
    fd1 = open_serial(device_name);

    if (fd1 < 0) {
        printf("Serial Fail\n");
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::spin(std::make_shared<MySubscriber>());
    rclcpp::shutdown();
    return 0;
}