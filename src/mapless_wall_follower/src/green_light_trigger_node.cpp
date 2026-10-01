// Start/stop trigger: GREEN enables the wall follower, a later RED disables it (latched).

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace mapless_wall_follower
{

class GreenLightTrigger : public rclcpp::Node
{
public:
  GreenLightTrigger()
  : Node("green_light_trigger")
  {
    green_topic_ = declare_parameter(
      "green_light_topic", std::string("/color_detector/green_light"));
    red_topic_ = declare_parameter(
      "red_light_topic", std::string("/color_detector/red_light"));
    enable_topic_ = declare_parameter(
      "enable_topic", std::string("/mapless_wall_follower/enable"));
    required_consecutive_ = declare_parameter("required_consecutive", 3);
    enable_publish_count_ = declare_parameter("enable_publish_count", 5);
    enable_publish_rate_ = declare_parameter("enable_publish_rate", 10.0);

    if (required_consecutive_ < 1 || enable_publish_count_ < 1 || enable_publish_rate_ <= 0.0) {
      throw std::invalid_argument("Invalid green light trigger parameters");
    }

    enable_pub_ = create_publisher<std_msgs::msg::Bool>(enable_topic_, 10);
    green_sub_ = create_subscription<std_msgs::msg::Bool>(
      green_topic_, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {onGreen(msg->data);});
    red_sub_ = create_subscription<std_msgs::msg::Bool>(
      red_topic_, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {onRed(msg->data);});

    // Enable subscriber is volatile: repeat a few times, only while it is connected.
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / enable_publish_rate_)),
      [this]() {publishEnable();});

    RCLCPP_INFO(
      get_logger(), "Waiting for GREEN on %s (%ld consecutive frames) -> enable %s; "
      "then RED on %s -> disable",
      green_topic_.c_str(), required_consecutive_, enable_topic_.c_str(), red_topic_.c_str());
  }

private:
  enum class State { WAITING, RUNNING, STOPPED };

  void onGreen(bool seen)
  {
    green_consecutive_ = seen ? green_consecutive_ + 1 : 0;
    if (state_ == State::WAITING && green_consecutive_ >= required_consecutive_) {
      state_ = State::RUNNING;
      send(true);
      RCLCPP_INFO(get_logger(), "GREEN LIGHT detected -> enabling wall follower");
    }
  }

  void onRed(bool seen)
  {
    red_consecutive_ = seen ? red_consecutive_ + 1 : 0;
    // RED before GREEN is the normal pre-start light, so only act once running.
    if (state_ == State::RUNNING && red_consecutive_ >= required_consecutive_) {
      state_ = State::STOPPED;
      send(false);
      RCLCPP_WARN(get_logger(), "RED LIGHT detected -> disabling wall follower (latched)");
    }
  }

  void send(bool value)
  {
    pending_value_ = value;
    sent_ = 0;
  }

  void publishEnable()
  {
    if (state_ == State::WAITING || sent_ >= enable_publish_count_) {
      return;
    }
    if (enable_pub_->get_subscription_count() == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "No subscriber on %s yet", enable_topic_.c_str());
      return;
    }
    std_msgs::msg::Bool msg;
    msg.data = pending_value_;
    enable_pub_->publish(msg);
    if (++sent_ >= enable_publish_count_) {
      RCLCPP_INFO(get_logger(), "Wall follower enable=%s sent", pending_value_ ? "true" : "false");
    }
  }

  std::string green_topic_;
  std::string red_topic_;
  std::string enable_topic_;
  int64_t required_consecutive_{3};
  int64_t enable_publish_count_{5};
  double enable_publish_rate_{10.0};

  State state_{State::WAITING};
  int64_t green_consecutive_{0};
  int64_t red_consecutive_{0};
  bool pending_value_{false};
  int64_t sent_{0};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr green_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr red_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mapless_wall_follower

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mapless_wall_follower::GreenLightTrigger>());
  rclcpp::shutdown();
  return 0;
}
