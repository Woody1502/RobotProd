#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ComPortEmulator : public rclcpp::Node
{
public:
  ComPortEmulator()
  : Node("com_port_emulator")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("serial_data", 10);
    timer_ = this->create_wall_timer(
      1s, std::bind(&ComPortEmulator::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "COM port emulator started");
  }

private:
  std::string read_from_com_port()
  {
    // Пока заглушка. Потом сюда  добавить реальное чтение из COM-порта.
    return "READ_SERIAL_DATA_12345";
  }

  void timer_callback()
  {
    std::string data = read_from_com_port();

    std_msgs::msg::String msg;
    msg.data = data;

    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Read from COM port: '%s'", data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComPortEmulator>());
  rclcpp::shutdown();
  return 0;
}
