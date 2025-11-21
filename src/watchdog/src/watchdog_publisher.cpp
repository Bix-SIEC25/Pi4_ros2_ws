#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/log_entry.hpp"

using namespace std::chrono_literals;

class WatchdogPublisher : public rclcpp::Node
{
public:
  WatchdogPublisher()
  : Node("watchdog_publisher")
  {
    publisher_ = this->create_publisher<interfaces::msg::LogEntry>("/logger", 10);

    timer_ = this->create_wall_timer(
      5s, std::bind(&WatchdogPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto msg = interfaces::msg::LogEntry();
    msg.level = 0;                         // TRACE as requested
    msg.sender = "watchdog";
    msg.message = "alive ping";

    publisher_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "[LEVEL(%u)] %s: %s",
                msg.level, msg.sender.c_str(), msg.message.c_str());
  }

  rclcpp::Publisher<interfaces::msg::LogEntry>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WatchdogPublisher>());
  rclcpp::shutdown();
  return 0;
}