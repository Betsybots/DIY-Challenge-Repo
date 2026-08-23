// Lightweight stub node used when CTRE Phoenix SDK is not available.
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("differential_drive_stub");
  RCLCPP_WARN(node->get_logger(), "Running differential-drive stub: Phoenix SDK not found. No hardware control will be performed.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
