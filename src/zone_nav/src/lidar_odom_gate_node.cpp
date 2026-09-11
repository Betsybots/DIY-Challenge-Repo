// ─────────────────────────────────────────────────────────────────────────────
// lidar_odom_gate_node.cpp
//
// Gates /lidar_odometry (FAST-LIO2 output) so EKF2 never receives corrupt
// lidar odometry while the robot is in the tunnel (BLIND_DRIVE state).
//
// WHY THIS EXISTS:
//   In the foil-lined tunnel, FAST-LIO2 loses all lidar returns and produces
//   garbage odometry (NaN poses, huge covariances, or wildly wrong positions).
//   EKF2 (robot_localization) has no built-in input-gating mechanism, so we
//   gate the topic here before it reaches EKF2.
//
// HOW IT WORKS:
//   • Subscribes to /lidar_odometry (FAST-LIO2 → raw)
//   • Subscribes to /nav_mode (zone_nav_manager → current state string)
//   • Republishes to /lidar_odometry_gated ONLY when nav_mode != "BLIND_DRIVE"
//   • EKF2's ekf_local.yaml uses odom1_topic: /lidar_odometry_gated
//
// IMPACT:
//   During BLIND_DRIVE, EKF2 receives no lidar input → runs on wheel+IMU only
//   (EKF1's /wimu_odom), which is exactly the doc's "EKF2 switches to
//   wheel+IMU only" requirement.
//   After BLIND_DRIVE, /lidar_odometry_gated resumes → EKF2 re-fuses lidar.
//
// WATCHDOG:
//   Bug42 fix: zone_nav_manager publishes /nav_mode at ~10 Hz. If it crashes
//   or is killed while the gate is CLOSED (BLIND_DRIVE), the gate would stay
//   permanently shut, depriving EKF2 of lidar and causing unbounded drift.
//   A wall timer checks for staleness every (timeout/2) seconds and reopens
//   the gate automatically if no /nav_mode message has arrived within
//   nav_mode_watchdog_timeout_s (default 5 s = 50 missed messages at 10 Hz).
//
// DEPENDENCIES: rclcpp, nav_msgs, std_msgs
// BUILD:  colcon build --packages-select diy_zone_nav
// ─────────────────────────────────────────────────────────────────────────────

#include <chrono>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class LidarOdomGateNode : public rclcpp::Node {
 public:
  LidarOdomGateNode() : Node("lidar_odom_gate") {
    declare_parameter<std::string>("lidar_odom_in",  "/lidar_odometry");
    declare_parameter<std::string>("lidar_odom_out", "/lidar_odometry_gated");
    declare_parameter<std::string>("nav_mode_topic", "/nav_mode");
    // Bug42 fix: watchdog timeout parameter.
    declare_parameter<double>("nav_mode_watchdog_timeout_s", 5.0);

    const std::string in_topic   = get_parameter("lidar_odom_in").as_string();
    const std::string out_topic  = get_parameter("lidar_odom_out").as_string();
    const std::string mode_topic = get_parameter("nav_mode_topic").as_string();

    double wd_timeout = get_parameter("nav_mode_watchdog_timeout_s").as_double();
    if (!(wd_timeout > 0.0)) {
      RCLCPP_WARN(get_logger(),
                  "nav_mode_watchdog_timeout_s must be > 0 (got %.3f) — using 5.0", wd_timeout);
      wd_timeout = 5.0;
    }
    watchdog_timeout_s_ = wd_timeout;

    pub_ = create_publisher<nav_msgs::msg::Odometry>(out_topic, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        in_topic, 10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          // Only forward when NOT in BLIND_DRIVE
          if (current_nav_mode_ != "BLIND_DRIVE") {
            pub_->publish(*msg);
          }
        });

    mode_sub_ = create_subscription<std_msgs::msg::String>(
        mode_topic, 10,
        [this](std_msgs::msg::String::SharedPtr msg) {
          last_mode_stamp_ = now();  // refresh watchdog on every received message
          if (msg->data != current_nav_mode_) {
            RCLCPP_INFO(get_logger(),
                        "nav_mode changed: %s → %s — lidar gate %s",
                        current_nav_mode_.c_str(),
                        msg->data.c_str(),
                        (msg->data == "BLIND_DRIVE") ? "CLOSED" : "OPEN");
            current_nav_mode_ = msg->data;
          }
        });

    // Bug42 fix: watchdog timer.
    // If zone_nav_manager crashes while the gate is CLOSED (BLIND_DRIVE),
    // current_nav_mode_ stays "BLIND_DRIVE" and EKF2 loses lidar permanently.
    // This timer checks at half the timeout period and reopens the gate when
    // /nav_mode goes stale, restoring lidar fusion automatically.
    last_mode_stamp_ = now();  // initialise AFTER Node is ready so now() is valid
    const auto wd_period = std::chrono::duration<double>(watchdog_timeout_s_ / 2.0);
    watchdog_timer_ = create_wall_timer(wd_period, [this]() {
      if (current_nav_mode_ != "BLIND_DRIVE") return;  // gate is open — nothing to do
      const double age = (now() - last_mode_stamp_).seconds();
      if (age > watchdog_timeout_s_) {
        RCLCPP_ERROR(get_logger(),
                     "nav_mode topic stale for %.1fs while gate is CLOSED "
                     "(zone_nav_manager crashed?) — reopening gate to prevent "
                     "permanent EKF2 lidar loss",
                     age);
        current_nav_mode_ = "WATCHDOG_OPEN";
      }
    });

    RCLCPP_INFO(get_logger(),
                "lidar_odom_gate ready: %s → [gate] → %s  (watchdog %.1fs)",
                in_topic.c_str(), out_topic.c_str(), watchdog_timeout_s_);
  }

 private:
  // safe default: gate open until we see "BLIND_DRIVE"
  std::string current_nav_mode_ = "INIT";

  // Bug42 fix: watchdog state
  rclcpp::Time last_mode_stamp_;   // initialised in constructor body after now() is valid
  double       watchdog_timeout_s_ = 5.0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdomGateNode>());
  rclcpp::shutdown();
  return 0;
}
