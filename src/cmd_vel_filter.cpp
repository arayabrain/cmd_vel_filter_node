#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

class CmdVelFilter : public rclcpp::Node
{
public:
  CmdVelFilter()
  : Node("cmd_vel_filter"), last_msg_time_(0)
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "raw_cmd_vel", 10, std::bind(&CmdVelFilter::cmd_vel_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // タイマーを設定（例：100msごとにチェック）
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&CmdVelFilter::timer_callback, this));

    timeout_duration_ = std::chrono::milliseconds(300); // タイムアウト時間を500msに設定

    RCLCPP_INFO(this->get_logger(), "CmdVelFilter node has been started.");
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_msg_time_ = this->now().nanoseconds();
    publisher_->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "Received and published cmd_vel");
  }

  void timer_callback()
  {
    auto current_time = this->now().nanoseconds();
    auto time_since_last_msg = current_time - last_msg_time_;

    if (time_since_last_msg > timeout_duration_.count() * 1000000)
    {
      // タイムアウト発生：停止信号を送信
      auto stop_msg = std::make_unique<geometry_msgs::msg::Twist>();
      stop_msg->linear.x = 0.0;
      stop_msg->linear.y = 0.0;
      stop_msg->linear.z = 0.0;
      stop_msg->angular.x = 0.0;
      stop_msg->angular.y = 0.0;
      stop_msg->angular.z = 0.0;

      publisher_->publish(std::move(stop_msg));
      RCLCPP_WARN(this->get_logger(), "No message received for %ld ms. Sending stop signal.", 
                  time_since_last_msg / 1000000);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int64_t last_msg_time_;
  std::chrono::milliseconds timeout_duration_;  

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelFilter>());
  rclcpp::shutdown();
  return 0;
}
