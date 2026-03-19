#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

bool stop_flag = false;

class key_subscriber : public rclcpp::Node
{
private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr key_mode_pub;

public:
key_subscriber()
:Node("key_subscriber")
{
    key_mode_pub = this->create_publisher<std_msgs::msg::Bool>("cmd_mode", 10);

}

void publish_key(std::string input_key)
  {
    auto control_mode = std_msgs::msg::Bool();

    if(input_key == "s"){
      stop_flag = false;
      RCLCPP_INFO(this->get_logger(), "Command: Navigation Start");
    }
    else if(input_key == "k"){
      stop_flag = true;
      RCLCPP_INFO(this->get_logger(), "Command: Navigation Stop");
    }
    else {
      RCLCPP_WARN(this->get_logger(), "Invalid input: '%s'. Please enter 's' or 'k'.", input_key.c_str());
      return; 
    }

    control_mode.data = stop_flag;
    // 入力内容を送信
    key_mode_pub->publish(control_mode);
  }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<key_subscriber>();

  std::string key_input;
  std::cout << "=== Keyboard Input Node ===" << std::endl;
  std::cout << " s : Navigation Start" << std::endl;
  std::cout << " k : Navigation Stop" << std::endl;
  std::cout << " Ctrl+C : Quit" << std::endl;
  std::cout << "===========================" << std::endl;

  while (rclcpp::ok()) {
    std::cout << "> "; 
    std::cin >> key_input; 

    node->publish_key(key_input);

  }

  rclcpp::shutdown();
  return 0;
}