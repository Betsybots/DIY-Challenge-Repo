// ─────────────────────────────────────────────────────────────────────────────
// zone_nav_manager_node.cpp
//
// Zone-aware navigation state machine for DIY Robot Challenge 2026.
//
// WHAT THIS NODE DOES:
//   • Reads a list of course zones from zone_waypoints.yaml (centre x,y + radius).
//   • Subscribes to /odometry/filtered (EKF2 output) to track robot position.
//   • When the robot enters a zone, transitions to that zone's nav mode:
//       NORMAL_NAV      — default Nav2 behaviour, full speed
//       SLOW_NAV        — reduced speed via /speed_limit
//       BLIND_DRIVE     — tunnel: gate EKF2 lidar input (via /nav_mode →
//                         lidar_odom_gate_node), switch mux to BLIND_DRIVE,
//                         publish /cmd_vel_zone_nav directly
//       NAVIGATE_AROUND — live costmap replanning (bucket obstacles), reduced speed
//       PUSH_THROUGH    — disable costmap obstacle layers (car-wash ribbons)
//   • Publishes /nav_mode (std_msgs/String) every tick so lidar_odom_gate_node
//     and other nodes react to state changes.
//   • Counts laps and stops at the finish line on the final lap.
//   • Waits for /green_light (std_msgs/Bool) before starting.
//
// ── HOW WE TALK TO NAV2 (and why) ────────────────────────────────────────────
//
// SPEED CONTROL — /speed_limit topic, NOT FollowPath.vx_max
//   Setting "FollowPath.vx_max" through /controller_server/set_parameters does
//   NOT work with the MPPI controller. In nav2_mppi_controller (humble):
//     optimizer.cpp:  getParam(s.base_constraints.vx_max, "vx_max", 0.5);
//   The dynamic-parameter callback writes `base_constraints`, but the actual
//   trajectory clipping uses `constraints`:
//     optimizer.cpp:  control_sequence_.vx = xt::clip(..., s.constraints.vx_max);
//   `constraints` is only re-synced from `base_constraints` inside reset() and
//   setSpeedLimit(). reset() only runs after `reset_period_` of INACTIVITY, so
//   a vx_max parameter write is silently ignored during an active run.
//
//   The supported runtime speed API is the /speed_limit topic
//   (nav2_msgs/SpeedLimit), handled by ControllerServer::speedLimitCallback →
//   controller->setSpeedLimit(). With percentage=false the value is absolute
//   m/s; a value of 0.0 (NO_SPEED_LIMIT) restores the configured maximum.
//
// COSTMAP PARAMETERS — note the DOUBLED namespace
//   nav2_costmap_2d::Costmap2DROS puts the costmap node inside its own
//   namespace (costmap_2d_ros.cpp):
//     __ns := add_namespaces(parent_namespace, local_namespace)
//   and planner_server.cpp / controller_server.cpp construct it as
//     Costmap2DROS("global_costmap", get_namespace(), "global_costmap")
//   so the fully-qualified node is /global_costmap/global_costmap and the
//   service is /global_costmap/global_costmap/set_parameters.
//   (This mirrors the doubled key nesting in nav2_params.yaml.)
//
// PUSH_THROUGH — disable the obstacle layers, do NOT touch obstacle heights
//   nav2_costmap_2d VoxelLayer (used by the local costmap) declares only
//   `max_obstacle_height` — there is NO `voxel_layer.min_obstacle_height`, so
//   a min-height trick cannot work on the local costmap at all.
//   Both ObstacleLayer and VoxelLayer DO support a dynamic `enabled` bool, so
//   we disable obstacle marking outright for the short car-wash zone and rely
//   on the reduced push speed. This matches the course design: the ribbons are
//   the only thing in that corridor and they are meant to be driven through.
//
// BLIND_DRIVE FLOW:
//   Entry: mux→BLIND_DRIVE | /nav_mode→BLIND_DRIVE gates EKF2 lidar input
//          | publish /cmd_vel_zone_nav at the timer rate
//   Exit:  NDT fitness recovered AND min_duration elapsed
//          → mux→AUTONOMOUS | stop /cmd_vel_zone_nav
//
// DEPENDENCIES: rclcpp, std_msgs, geometry_msgs, nav_msgs, nav2_msgs, rcl_interfaces
// BUILD:  colcon build --packages-select diy_zone_nav
// RUN:    ros2 launch diy_zone_nav zone_nav.launch.py
// ─────────────────────────────────────────────────────────────────────────────

#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// Nav2 uses 0.0 on /speed_limit to mean "no limit — restore configured maximum".
// (nav2_costmap_2d::NO_SPEED_LIMIT; inlined here to avoid the extra dependency.)
static constexpr double kNoSpeedLimit = 0.0;

// How often enforceMuxMode() may re-send the mode request while the mux still
// disagrees with us (seconds).
static constexpr double kMuxReassertPeriodS = 0.5;

// Mux mode strings — must match diy_cmd_vel_mux Mode class values exactly.
static const char * kMuxAutonomous = "AUTONOMOUS";
static const char * kMuxBlindDrive = "BLIND_DRIVE";
static const char * kMuxEstopLock  = "ESTOP_LOCK";

// ──────────────────────────────────────────────────────────────────────────────
// Data structures
// ──────────────────────────────────────────────────────────────────────────────

enum class NavMode {
  INIT,            // waiting for green light — robot stationary
  NORMAL_NAV,      // default Nav2, full speed
  SLOW_NAV,        // reduced speed (ramp, narrow, helix, bank turn, gravel)
  BLIND_DRIVE,     // wheel+IMU dead reckoning in tunnel — no lidar, no Nav2
  NAVIGATE_AROUND, // live costmap replanning (bucket obstacles), reduced speed
  PUSH_THROUGH,    // obstacle layers disabled to ignore car-wash ribbons
  DONE             // finished all laps — stop
};

static std::string modeToString(NavMode m) {
  switch (m) {
    case NavMode::INIT:            return "INIT";
    case NavMode::NORMAL_NAV:      return "NORMAL_NAV";
    case NavMode::SLOW_NAV:        return "SLOW_NAV";
    case NavMode::BLIND_DRIVE:     return "BLIND_DRIVE";
    case NavMode::NAVIGATE_AROUND: return "NAVIGATE_AROUND";
    case NavMode::PUSH_THROUGH:    return "PUSH_THROUGH";
    case NavMode::DONE:            return "DONE";
  }
  return "UNKNOWN";
}

// One entry from zone_waypoints.yaml
struct Zone {
  std::string label;
  double      x;
  double      y;
  double      radius;            // enter mode when closer than this
  double      exit_radius;       // leave mode when farther than this (hysteresis)
  NavMode     mode;
  bool        mode_set;          // true only when `mode:` key was parsed — catch omissions
  double      inflation_radius;  // >0 → override costmap inflation on entry/exit
  double      ndt_fitness_gate;  // >0 → log WARN if NDT fitness worse than this on entry
  Zone() : x(0), y(0), radius(1.0), exit_radius(2.0),
           mode(NavMode::NORMAL_NAV), mode_set(false),
           inflation_radius(-1.0), ndt_fitness_gate(-1.0) {}
};

// Finish line from zone_waypoints.yaml
struct FinishLine {
  double x      = 0.0;
  double y      = 0.0;
  double radius = 1.0;
};

// ──────────────────────────────────────────────────────────────────────────────
// ZoneNavManagerNode
// ──────────────────────────────────────────────────────────────────────────────

class ZoneNavManagerNode : public rclcpp::Node {
 public:
  ZoneNavManagerNode() : Node("zone_nav_manager") {
    // ── Declare + read all parameters ──────────────────────────────────────
    declare_parameter<std::string>("zone_waypoints_file", "");
    declare_parameter<double>("update_rate_hz", 10.0);
    declare_parameter<double>("normal_vx_max", 0.6);
    declare_parameter<double>("slow_vx_max", 0.20);
    declare_parameter<double>("navigate_around_vx_max", 0.30);
    declare_parameter<double>("push_vx_max", 0.20);
    declare_parameter<double>("blind_drive_vx", 0.18);
    declare_parameter<double>("blind_drive_min_duration_s", 2.0);
    declare_parameter<double>("blind_drive_max_duration_s", 15.0);
    declare_parameter<double>("blind_drive_exit_fitness_threshold", 0.8);
    declare_parameter<double>("default_inflation_radius", 0.45);
    declare_parameter<double>("local_default_inflation_radius", 0.40);
    // Full service names. These MUST include the doubled costmap namespace —
    // see the header comment. Exposed as parameters so they can be corrected
    // from YAML if Nav2 is ever launched inside a non-root namespace.
    declare_parameter<std::string>("global_costmap_param_service",
                                   "/global_costmap/global_costmap/set_parameters");
    declare_parameter<std::string>("local_costmap_param_service",
                                   "/local_costmap/local_costmap/set_parameters");
    declare_parameter<std::string>("mux_param_service",
                                   "/cmd_vel_mux_node/set_parameters");
    declare_parameter<std::string>("speed_limit_topic", "/speed_limit");
    declare_parameter<std::string>("mux_mode_topic", "/mux_mode");
    declare_parameter<std::string>("start_signal_topic", "/green_light");
    declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    declare_parameter<std::string>("ndt_fitness_topic", "/ndt_fitness_score");
    declare_parameter<std::string>("nav_mode_topic", "/nav_mode");
    declare_parameter<bool>("verbose", true);
    declare_parameter<int>("total_laps", 2);

    normal_vx_max_          = get_parameter("normal_vx_max").as_double();
    slow_vx_max_            = get_parameter("slow_vx_max").as_double();
    navigate_around_vx_max_ = get_parameter("navigate_around_vx_max").as_double();
    push_vx_max_            = get_parameter("push_vx_max").as_double();
    blind_vx_               = get_parameter("blind_drive_vx").as_double();
    blind_min_dur_          = get_parameter("blind_drive_min_duration_s").as_double();
    blind_max_dur_          = get_parameter("blind_drive_max_duration_s").as_double();
    blind_fitness_exit_thr_ = get_parameter("blind_drive_exit_fitness_threshold").as_double();
    total_laps_             = static_cast<int>(get_parameter("total_laps").as_int());
    verbose_                = get_parameter("verbose").as_bool();
    default_inflation_radius_       = get_parameter("default_inflation_radius").as_double();
    local_default_inflation_radius_ = get_parameter("local_default_inflation_radius").as_double();

    const std::string global_costmap_srv = get_parameter("global_costmap_param_service").as_string();
    const std::string local_costmap_srv  = get_parameter("local_costmap_param_service").as_string();
    const std::string mux_srv            = get_parameter("mux_param_service").as_string();
    const std::string speed_limit_topic  = get_parameter("speed_limit_topic").as_string();
    const std::string mux_mode_topic     = get_parameter("mux_mode_topic").as_string();
    const std::string odom_topic         = get_parameter("odom_topic").as_string();
    const std::string fitness_topic      = get_parameter("ndt_fitness_topic").as_string();
    const std::string nav_mode_topic     = get_parameter("nav_mode_topic").as_string();
    const std::string start_topic        = get_parameter("start_signal_topic").as_string();

    // ── Load zone waypoints ────────────────────────────────────────────────
    // Throw rather than calling rclcpp::shutdown() from the constructor: the
    // object would still be handed to rclcpp::spin(), which then operates on a
    // shut-down context. main() catches this and exits with a non-zero code.
    const std::string waypoints_file = get_parameter("zone_waypoints_file").as_string();
    if (waypoints_file.empty()) {
      throw std::runtime_error("zone_waypoints_file parameter is empty. Cannot start.");
    }
    if (!loadZoneWaypoints(waypoints_file)) {
      throw std::runtime_error("Failed to load zone waypoints from: " + waypoints_file);
    }
    if (total_laps_ < 1) {
      throw std::runtime_error("total_laps must be >= 1");
    }
    if (!(blind_max_dur_ > blind_min_dur_)) {
      throw std::runtime_error("blind_drive_max_duration_s must be > blind_drive_min_duration_s");
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu zones from %s", zones_.size(), waypoints_file.c_str());

    // ── Publishers ────────────────────────────────────────────────────────
    nav_mode_pub_     = create_publisher<std_msgs::msg::String>(nav_mode_topic, 10);
    // /cmd_vel_zone_nav: published during BLIND_DRIVE and DONE only.
    // cmd_vel_mux_node in BLIND_DRIVE mode forwards this → /cmd_vel_safe → motors.
    cmd_vel_zone_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_zone_nav", 10);
    speed_limit_pub_  = create_publisher<nav2_msgs::msg::SpeedLimit>(speed_limit_topic, 10);

    // ── Subscribers ──────────────────────────────────────────────────────
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(msg); });

    fitness_sub_ = create_subscription<std_msgs::msg::Float32>(
        fitness_topic, 10,
        [this](std_msgs::msg::Float32::SharedPtr msg) { latest_fitness_ = msg->data; });

    // Mux mode feedback. Needed so we can re-assert the mode we want: the mux
    // boots in JOYSTICK and also reverts to JOYSTICK whenever an E-stop is
    // released. Without this feedback loop the robot silently never moves.
    mux_mode_sub_ = create_subscription<std_msgs::msg::String>(
        mux_mode_topic, 10,
        [this](std_msgs::msg::String::SharedPtr msg) {
          reported_mux_mode_ = msg->data;
          mux_mode_seen_     = true;
        });

    green_light_sub_ = create_subscription<std_msgs::msg::Bool>(
        start_topic, 10,
        [this](std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data && current_mode_ == NavMode::INIT) {
            RCLCPP_INFO(get_logger(), "GREEN LIGHT — starting race!");
            transitionTo(NavMode::NORMAL_NAV);
          }
        });

    // ── SetParameters clients ─────────────────────────────────────────────
    costmap_param_client_       = create_client<rcl_interfaces::srv::SetParameters>(global_costmap_srv);
    local_costmap_param_client_ = create_client<rcl_interfaces::srv::SetParameters>(local_costmap_srv);
    mux_param_client_           = create_client<rcl_interfaces::srv::SetParameters>(mux_srv);

    // ── State machine update timer ────────────────────────────────────────
    double rate_hz = get_parameter("update_rate_hz").as_double();
    if (!(rate_hz > 0.0)) {
      RCLCPP_WARN(get_logger(), "update_rate_hz must be > 0 (got %.3f) — using 10.0", rate_hz);
      rate_hz = 10.0;
    }
    tick_period_s_ = 1.0 / rate_hz;
    auto period    = std::chrono::duration<double>(tick_period_s_);
    update_timer_  = create_wall_timer(period, [this]() { onUpdateTimer(); });

    RCLCPP_INFO(get_logger(), "ZoneNavManager ready — waiting for green light on %s",
                start_topic.c_str());
  }

 private:
  // ── State ────────────────────────────────────────────────────────────────
  NavMode current_mode_    = NavMode::INIT;
  int     active_zone_idx_ = -1;  // index into zones_ while in a zone, -1 if none
  // Zone we exited early (BLIND_DRIVE fitness exit) while still geometrically
  // inside its entry radius. Suppressed from re-entry until we clear its
  // exit_radius, otherwise the next tick would immediately re-trigger it.
  int     suppressed_zone_idx_ = -1;
  int     current_lap_     = 0;
  // Start true so the robot's initial position at the start line is not counted
  // as a lap crossing. The robot must leave and return before lap 1 is counted.
  bool    in_finish_zone_  = true;
  bool    odom_received_   = false;
  // False when zone_waypoints.yaml has no finish_line block. Lap counting is
  // then genuinely disabled instead of silently using the (0,0) default, which
  // would count spurious laps every time the robot passed near the map origin.
  bool    finish_line_valid_ = false;
  rclcpp::Time last_odom_time_;

  // Robot position (updated from /odometry/filtered)
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  double robot_yaw_ = 0.0;

  // BLIND_DRIVE timing
  rclcpp::Time blind_entry_time_;
  bool         blind_min_elapsed_ = false;

  // NDT fitness (updated from subscriber)
  float latest_fitness_ = 999.0f;

  // Mux arbitration. desired_mux_mode_ empty → we don't care (INIT: leave the
  // operator in control). Otherwise re-asserted until the mux reports it.
  std::string desired_mux_mode_;
  std::string reported_mux_mode_;
  bool        mux_mode_seen_ = false;

  // Speed limit currently requested on /speed_limit (absolute m/s, 0.0 = none).
  double desired_speed_limit_ = kNoSpeedLimit;

  double tick_period_s_ = 0.1;
  double reassert_accum_s_ = 0.0;
  // enforceMuxMode() is rate-limited so a mux that is down or rejecting our
  // request does not get hammered at the full tick rate.
  double mux_reassert_accum_s_ = 0.0;

  // Params
  double normal_vx_max_;
  double slow_vx_max_;
  double navigate_around_vx_max_;
  double push_vx_max_;
  double blind_vx_;
  double blind_min_dur_;
  double blind_max_dur_;
  double blind_fitness_exit_thr_;
  double default_inflation_radius_;
  double local_default_inflation_radius_;
  int    total_laps_;
  bool   verbose_;

  // Zones + finish
  std::vector<Zone> zones_;
  FinishLine        finish_line_;

  // ROS handles
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_mode_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_zone_pub_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr fitness_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mux_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr green_light_sub_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr costmap_param_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr local_costmap_param_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr mux_param_client_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  // ── Odometry callback ────────────────────────────────────────────────────
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr & msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    const auto & q = msg->pose.pose.orientation;
    robot_yaw_     = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    odom_received_  = true;
    last_odom_time_ = now();
  }

  // ── Main state machine update (called by timer) ───────────────────────────
  void onUpdateTimer() {
    // Always publish current mode for other nodes (lidar_odom_gate_node).
    publishNavMode();
    // Keep the mux and the controller speed limit pinned to what we want,
    // regardless of restarts, E-stop releases, or late-joining nodes.
    enforceMuxMode();
    reassertSpeedLimit();

    if (current_mode_ == NavMode::INIT) {
      return;  // waiting for green light
    }

    if (current_mode_ == NavMode::DONE) {
      // Keep publishing zero so the mux forwards a live stop rather than
      // relying on its staleness timeout.
      stopBlindDriveCmdVel();
      return;
    }

    // Zone geometry is meaningless until we have a real pose.
    if (!odom_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "No odometry received yet — zone evaluation paused");
      return;
    }
    // Keep acting on the last known pose if odometry stalls (freezing the state
    // machine would be worse than acting on a slightly stale pose), but say so
    // loudly: every zone and lap decision below is now based on old data.
    const double odom_age = (now() - last_odom_time_).seconds();
    if (odom_age > 1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Odometry is %.1fs stale — zone decisions use the last known pose",
                           odom_age);
    }

    // ── BLIND_DRIVE ───────────────────────────────────────────────────────
    // Stay blind until min duration AND NDT fitness recovers.
    if (current_mode_ == NavMode::BLIND_DRIVE) {
      publishBlindDriveCmdVel();

      const double elapsed = (now() - blind_entry_time_).seconds();
      if (elapsed >= blind_min_dur_) {
        blind_min_elapsed_ = true;
      }

      // Failsafe: if NDT never recovers (tunnel longer than expected, NDT node
      // dead, map mismatch) we must not dead-reckon forward forever.
      const bool timed_out = elapsed >= blind_max_dur_;
      if (timed_out) {
        RCLCPP_ERROR(get_logger(),
                     "BLIND_DRIVE TIMEOUT after %.1fs (max %.1fs) — NDT fitness still %.3f. "
                     "Handing control back to Nav2; localisation may be poor.",
                     elapsed, blind_max_dur_, latest_fitness_);
      }

      if (timed_out || (blind_min_elapsed_ && latest_fitness_ < blind_fitness_exit_thr_)) {
        if (!timed_out) {
          RCLCPP_INFO(get_logger(),
                      "BLIND_DRIVE exit — NDT fitness %.3f < threshold %.3f after %.1fs",
                      latest_fitness_, blind_fitness_exit_thr_, elapsed);
        }
        // We are usually still inside the tunnel-entry zone radius here, so
        // suppress that zone until we have physically cleared it.
        suppressed_zone_idx_ = active_zone_idx_;
        // Bug41 fix: use the correct reason string for both exit paths so the
        // log is not misleading when the timeout (not fitness recovery) fires.
        exitActiveZone(timed_out ? "BLIND_DRIVE timeout" : "NDT fitness recovered");
        transitionTo(NavMode::NORMAL_NAV);
      }
      // Bug40 fix: check the finish line even while in BLIND_DRIVE.
      // If the tunnel is near the start/finish line the robot can cross the
      // line while blind.  Without this call the lap crossing is silently
      // dropped, and on the final lap DONE is never triggered — the robot
      // continues driving forever.
      checkFinishLine();
      return;  // don't evaluate other zones while blind
    }

    // ── Clear zone-re-entry suppression once we're genuinely clear ────────
    if (suppressed_zone_idx_ >= 0) {
      const Zone & sz = zones_[suppressed_zone_idx_];
      if (distanceTo(sz.x, sz.y) > sz.exit_radius) {
        if (verbose_) {
          RCLCPP_INFO(get_logger(), "Cleared zone [%s] — re-entry no longer suppressed",
                      sz.label.c_str());
        }
        suppressed_zone_idx_ = -1;
      }
    }

    if (active_zone_idx_ < 0) {
      // ── Zone entry check ────────────────────────────────────────────────
      // Pick the NEAREST matching zone, not the first one in file order:
      // trigger radii overlap (e.g. tunnel_exit and narrow_section), and
      // file order is arbitrary, so first-match would enter the wrong zone.
      int    best_idx  = -1;
      double best_dist = 0.0;
      for (int i = 0; i < static_cast<int>(zones_.size()); ++i) {
        if (i == suppressed_zone_idx_) continue;
        // Bug44 fix: skip zones whose mode matches the current mode.
        // A zone with mode==current_mode_ (e.g. a NORMAL_NAV marker zone)
        // would call transitionTo() as a noop but still set active_zone_idx_,
        // blocking ALL other zone entry triggers until the robot exits the
        // marker zone's exit_radius. For the tunnel_exit marker specifically
        // this can prevent narrow_section from triggering at full approach speed.
        if (zones_[i].mode == current_mode_) continue;
        const double d = distanceTo(zones_[i].x, zones_[i].y);
        if (d < zones_[i].radius && (best_idx < 0 || d < best_dist)) {
          best_idx  = i;
          best_dist = d;
        }
      }
      if (best_idx >= 0) {
        enterZone(best_idx);
        // fall through to the lap check — do NOT return
      }
    } else {
      // ── Zone exit check (exit_radius gives hysteresis) ──────────────────
      const Zone & zone = zones_[active_zone_idx_];
      if (distanceTo(zone.x, zone.y) > zone.exit_radius) {
        exitActiveZone("left zone radius");
        transitionTo(NavMode::NORMAL_NAV);
      }
    }

    // ── Finish line + lap counting ────────────────────────────────────────
    // Runs on every tick, including the tick a zone was entered on, so a lap
    // crossing that coincides with a zone trigger is never dropped.
    checkFinishLine();
  }

  // ── Enter a zone ─────────────────────────────────────────────────────────
  void enterZone(int idx) {
    const Zone & zone = zones_[idx];
    if (verbose_) {
      RCLCPP_INFO(get_logger(), "Entered zone [%s] → mode: %s",
                  zone.label.c_str(), modeToString(zone.mode).c_str());
    }
    active_zone_idx_ = idx;

    // NDT fitness gate for zones that require good localisation.
    // The narrow section needs <4cm lateral accuracy per the design doc (§5.2).
    if (zone.ndt_fitness_gate > 0.0) {
      if (latest_fitness_ > zone.ndt_fitness_gate) {
        RCLCPP_WARN(get_logger(),
                    "ZONE [%s]: NDT fitness %.3f > gate %.3f — localisation is weak! "
                    "Entering anyway but lateral accuracy may be insufficient.",
                    zone.label.c_str(), latest_fitness_, zone.ndt_fitness_gate);
      } else {
        RCLCPP_INFO(get_logger(),
                    "ZONE [%s]: NDT fitness %.3f — localisation OK (gate %.3f)",
                    zone.label.c_str(), latest_fitness_, zone.ndt_fitness_gate);
      }
    }

    // Per-zone inflation_radius override. Both costmaps must be reduced: the
    // local costmap governs real-time collision checking, so leaving it at
    // 0.40m keeps the narrow-section walls lethal even if the global is fixed.
    if (zone.inflation_radius > 0.0) {
      RCLCPP_INFO(get_logger(),
                  "  Reducing inflation_radius: global %.2f→%.2f m  local %.2f→%.2f m",
                  default_inflation_radius_, zone.inflation_radius,
                  local_default_inflation_radius_, zone.inflation_radius);
      setInflationRadius(zone.inflation_radius, zone.inflation_radius);
    }

    transitionTo(zone.mode);
  }

  // ── Leave the active zone (single place that undoes zone-scoped changes) ──
  // Called both by the normal geometric exit and by the BLIND_DRIVE fitness
  // exit, so an inflation override can never leak past the zone that set it.
  void exitActiveZone(const char * reason) {
    if (active_zone_idx_ < 0) return;
    const Zone & zone = zones_[active_zone_idx_];

    if (verbose_) {
      RCLCPP_INFO(get_logger(), "Exited zone [%s] (%s)", zone.label.c_str(), reason);
    }

    if (zone.inflation_radius > 0.0) {
      RCLCPP_INFO(get_logger(),
                  "  Restoring inflation_radius: global→%.2f m  local→%.2f m",
                  default_inflation_radius_, local_default_inflation_radius_);
      setInflationRadius(default_inflation_radius_, local_default_inflation_radius_);
    }

    active_zone_idx_ = -1;
  }

  // ── State transition ─────────────────────────────────────────────────────
  void transitionTo(NavMode new_mode) {
    if (new_mode == current_mode_) return;

    if (verbose_) {
      RCLCPP_INFO(get_logger(), "Mode: %s → %s",
                  modeToString(current_mode_).c_str(),
                  modeToString(new_mode).c_str());
    }

    handleModeExit(current_mode_);
    current_mode_ = new_mode;
    handleModeEntry(new_mode);

    // Publish immediately so lidar_odom_gate_node reacts without waiting a tick.
    publishNavMode();
  }

  // Called when LEAVING a mode — clean up that mode's side effects
  void handleModeExit(NavMode old_mode) {
    switch (old_mode) {
      case NavMode::PUSH_THROUGH:
        // Re-enable obstacle marking — Nav2 resumes normal collision avoidance.
        setObstacleLayersEnabled(true);
        break;

      case NavMode::BLIND_DRIVE:
        // Stop publishing /cmd_vel_zone_nav. The mux returns to AUTONOMOUS via
        // handleModeEntry of the next mode + enforceMuxMode().
        stopBlindDriveCmdVel();
        blind_min_elapsed_ = false;
        break;

      default:
        break;
    }
    // Speed limits are always re-established by handleModeEntry, so there is
    // deliberately no per-mode speed restoration here.
  }

  // Called when ENTERING a mode — apply that mode's side effects
  void handleModeEntry(NavMode new_mode) {
    switch (new_mode) {
      case NavMode::NORMAL_NAV:
        requestMuxMode(kMuxAutonomous);
        // 0.0 = NO_SPEED_LIMIT → MPPI restores its configured vx_max.
        requestSpeedLimit(kNoSpeedLimit);
        break;

      case NavMode::SLOW_NAV:
        requestMuxMode(kMuxAutonomous);
        requestSpeedLimit(slow_vx_max_);
        break;

      case NavMode::NAVIGATE_AROUND:
        // Reduce speed for dynamic replanning around randomly placed buckets:
        // at 0.6 m/s Nav2 may not replan fast enough at close range.
        requestMuxMode(kMuxAutonomous);
        requestSpeedLimit(navigate_around_vx_max_);
        break;

      case NavMode::PUSH_THROUGH:
        requestMuxMode(kMuxAutonomous);
        requestSpeedLimit(push_vx_max_);
        // Disable obstacle marking so the hanging ribbons, which read as a
        // solid wall to the lidar, do not block the plan. Safe only because
        // this zone is short and the push speed is low.
        setObstacleLayersEnabled(false);
        break;

      case NavMode::BLIND_DRIVE:
        // Take the mux first so the very first command we publish is forwarded.
        requestMuxMode(kMuxBlindDrive);
        enforceMuxMode();
        blind_entry_time_  = now();
        blind_min_elapsed_ = false;
        latest_fitness_    = 999.0f;  // reset so we don't exit on stale data
        RCLCPP_WARN(get_logger(),
                    "BLIND_DRIVE entered — robot_yaw %.3f rad | mux→BLIND_DRIVE | "
                    "EKF2 lidar gated | /cmd_vel_zone_nav active",
                    robot_yaw_);
        break;

      case NavMode::DONE:
        RCLCPP_INFO(get_logger(), "RACE COMPLETE — %d laps done. Stopping.", current_lap_);
        // We may be standing inside a zone that overrode inflation_radius;
        // put the costmaps back so a post-race manual drive is not affected.
        exitActiveZone("race complete");
        // Restore full speed so a Nav2 restart post-race (debug/inspection run)
        // does not inherit the last zone's speed cap.
        requestSpeedLimit(kNoSpeedLimit);
        // Take the mux so our zero-velocity command actually reaches the motors;
        // in AUTONOMOUS the mux would keep forwarding Nav2 instead.
        requestMuxMode(kMuxBlindDrive);
        enforceMuxMode();
        stopBlindDriveCmdVel();
        break;

      default:
        break;
    }
  }

  // ── Finish line / lap counting ───────────────────────────────────────────
  void checkFinishLine() {
    if (!finish_line_valid_) return;   // no finish_line in the waypoints file
    const bool near_finish = distanceTo(finish_line_.x, finish_line_.y) < finish_line_.radius;

    if (near_finish && !in_finish_zone_) {
      in_finish_zone_ = true;
      current_lap_++;
      RCLCPP_INFO(get_logger(), "Lap %d/%d complete!", current_lap_, total_laps_);
      if (current_lap_ >= total_laps_) {
        transitionTo(NavMode::DONE);
      }
    } else if (!near_finish) {
      in_finish_zone_ = false;  // reset so we can count the next pass
    }
  }

  // ── Speed limit (the ONLY working runtime speed API for MPPI) ────────────
  void requestSpeedLimit(double vx_max_abs) {
    desired_speed_limit_ = vx_max_abs;
    publishSpeedLimit();
    if (verbose_) {
      if (vx_max_abs == kNoSpeedLimit) {
        RCLCPP_INFO(get_logger(), "  speed limit → none (restore %.2f m/s)", normal_vx_max_);
      } else {
        RCLCPP_INFO(get_logger(), "  speed limit → %.2f m/s", vx_max_abs);
      }
    }
  }

  void publishSpeedLimit() {
    nav2_msgs::msg::SpeedLimit msg;
    msg.header.stamp = now();
    msg.percentage   = false;          // absolute m/s
    msg.speed_limit  = desired_speed_limit_;
    speed_limit_pub_->publish(msg);
  }

  // controller_server subscribes with volatile QoS, so a limit published
  // before it came up would be lost. Re-send once a second; setSpeedLimit()
  // is idempotent so this is safe to repeat.
  void reassertSpeedLimit() {
    reassert_accum_s_ += tick_period_s_;
    if (reassert_accum_s_ >= 1.0) {
      reassert_accum_s_ = 0.0;
      publishSpeedLimit();
    }
  }

  // ── BLIND_DRIVE cmd_vel publisher ────────────────────────────────────────
  void publishBlindDriveCmdVel() {
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = blind_vx_;
    twist.angular.z = 0.0;  // straight — heading maintained by EKF1 wheel+IMU
    cmd_vel_zone_pub_->publish(twist);
  }

  void stopBlindDriveCmdVel() {
    cmd_vel_zone_pub_->publish(geometry_msgs::msg::Twist());
  }

  // ── Costmap obstacle layer enable/disable (PUSH_THROUGH) ─────────────────
  // The global costmap uses ObstacleLayer, the local costmap uses VoxelLayer,
  // so the parameter prefixes differ. Both support `enabled` as a dynamic
  // parameter (nav2_costmap_2d obstacle_layer.cpp / voxel_layer.cpp).
  void setObstacleLayersEnabled(bool enabled) {
    RCLCPP_INFO(get_logger(), "  costmap obstacle layers → %s",
                enabled ? "ENABLED" : "DISABLED");
    sendBoolParam(costmap_param_client_,       "global_costmap", "obstacle_layer.enabled", enabled);
    sendBoolParam(local_costmap_param_client_, "local_costmap",  "voxel_layer.enabled",    enabled);
  }

  // ── Costmap inflation radius ─────────────────────────────────────────────
  // inflation_layer.inflation_radius is dynamically reconfigurable on both
  // costmaps; InflationLayer::dynamicParametersCallback calls matchSize() and
  // sets need_reinflation_, so the change takes effect on the next update.
  void setInflationRadius(double global_radius, double local_radius) {
    sendDoubleParam(costmap_param_client_,       "global_costmap",
                    "inflation_layer.inflation_radius", global_radius);
    sendDoubleParam(local_costmap_param_client_, "local_costmap",
                    "inflation_layer.inflation_radius", local_radius);
  }

  // ── Generic SetParameters helpers ────────────────────────────────────────
  void sendDoubleParam(
      const rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr & client,
      const std::string & who, const std::string & param_name, double value) {
    rcl_interfaces::msg::Parameter param;
    param.name               = param_name;
    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param.value.double_value = value;
    sendParam(client, who, param);
  }

  void sendBoolParam(
      const rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr & client,
      const std::string & who, const std::string & param_name, bool value) {
    rcl_interfaces::msg::Parameter param;
    param.name             = param_name;
    param.value.type       = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    param.value.bool_value = value;
    sendParam(client, who, param);
  }

  void sendParam(
      const rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr & client,
      const std::string & who, const rcl_interfaces::msg::Parameter & param) {
    if (!client->service_is_ready()) {
      RCLCPP_ERROR(get_logger(),
                   "%s SetParameters service (%s) not available — '%s' NOT applied!",
                   who.c_str(), client->get_service_name(), param.name.c_str());
      return;
    }
    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    req->parameters.push_back(param);
    // Check the reply: Nav2 silently reports failure for unknown parameters,
    // which is exactly how a typo'd parameter name becomes an invisible bug.
    //
    // Capture a WEAK pointer, not raw `this`. If the node is destroyed while a
    // service response is in flight (e.g., Ctrl-C just after the finish line),
    // the callback locks the weak_ptr; if it has expired the callback returns
    // cleanly instead of dereferencing a dangling pointer (UAF / crash).
    std::weak_ptr<ZoneNavManagerNode> weak_self =
        std::static_pointer_cast<ZoneNavManagerNode>(shared_from_this());
    client->async_send_request(
        req,
        [weak_self, who, name = param.name](
            rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
          auto self = weak_self.lock();
          if (!self) return;   // node already destroyed — discard the response
          const auto results = future.get()->results;
          if (results.empty() || !results[0].successful) {
            RCLCPP_ERROR(self->get_logger(), "%s rejected parameter '%s': %s",
                         who.c_str(), name.c_str(),
                         results.empty() ? "no result" : results[0].reason.c_str());
          }
        });
  }

  // ── Mux mode arbitration ─────────────────────────────────────────────────
  void requestMuxMode(const std::string & mode) {
    if (desired_mux_mode_ != mode) {
      RCLCPP_INFO(get_logger(), "  cmd_vel_mux mode → %s", mode.c_str());
      // Pre-load the rate-limiter accumulator so the next enforceMuxMode() call
      // fires immediately rather than waiting up to kMuxReassertPeriodS.
      // This matters most at race start (INIT → NORMAL_NAV via green light),
      // where the accumulator is at 0 because INIT never sent any mux request.
      mux_reassert_accum_s_ = kMuxReassertPeriodS;
    }
    desired_mux_mode_ = mode;
  }

  // Re-assert the desired mux mode whenever the mux disagrees. This covers:
  //   • the mux booting in JOYSTICK while we are already in NORMAL_NAV
  //   • the mux reverting to JOYSTICK after an E-stop is released mid-race
  //   • the mux node restarting
  // ESTOP_LOCK is never overridden — the mux rejects that anyway, and the
  // E-stop must only be cleared by the hardware signal.
  void enforceMuxMode() {
    if (desired_mux_mode_.empty()) return;                     // INIT: leave operator in control
    if (reported_mux_mode_ == kMuxEstopLock) return;           // respect the E-stop
    if (mux_mode_seen_ && reported_mux_mode_ == desired_mux_mode_) {
      mux_reassert_accum_s_ = kMuxReassertPeriodS;             // ready to fire immediately
      return;
    }
    // Rate-limit: the mux takes a moment to apply and echo back the change, and
    // if it is down or rejecting we must not spam it at the full tick rate.
    mux_reassert_accum_s_ += tick_period_s_;
    if (mux_reassert_accum_s_ < kMuxReassertPeriodS) return;
    mux_reassert_accum_s_ = 0.0;
    if (!mux_param_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "cmd_vel_mux SetParameters (%s) not available — cannot set mode %s",
                           mux_param_client_->get_service_name(), desired_mux_mode_.c_str());
      return;
    }
    rcl_interfaces::msg::Parameter param;
    param.name               = "mode";
    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    param.value.string_value = desired_mux_mode_;
    auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    req->parameters.push_back(param);
    mux_param_client_->async_send_request(req);
  }

  // ── Nav mode publisher ───────────────────────────────────────────────────
  void publishNavMode() {
    std_msgs::msg::String msg;
    msg.data = modeToString(current_mode_);
    nav_mode_pub_->publish(msg);
  }

  // ── Utility: distance from robot to a point ──────────────────────────────
  double distanceTo(double x, double y) const {
    const double dx = robot_x_ - x;
    const double dy = robot_y_ - y;
    return std::sqrt(dx * dx + dy * dy);
  }

  // ── Zone waypoints loader ────────────────────────────────────────────────
  // Minimal hand-rolled parser so yaml-cpp is not needed.
  // Expected format: a "zones:" list of "- label: ..." blocks, plus
  // "finish_line: { x, y, radius }" and "total_laps: N".
  bool loadZoneWaypoints(const std::string & filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s", filepath.c_str());
      return false;
    }

    Zone current_zone;
    bool in_zones_block  = false;
    bool in_zone_entry   = false;
    bool in_finish_block = false;
    bool finish_has_x_      = false;
    bool finish_has_y_      = false;
    bool finish_has_radius_ = false;

    // Strip leading whitespace, any trailing "# comment", and trailing space.
    // Returns "" for blank lines and for whole-line comments — including a '#'
    // in column 0, which an earlier version mishandled by computing (hash - 1)
    // on an unsigned index and reading s[SIZE_MAX].
    auto trim = [](const std::string & s) -> std::string {
      const size_t start = s.find_first_not_of(" \t\r\n");
      if (start == std::string::npos) return "";
      const size_t hash = s.find('#', start);
      size_t end = (hash == std::string::npos) ? s.size() : hash;  // exclusive
      while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
        --end;
      }
      if (end <= start) return "";
      return s.substr(start, end - start);
    };

    bool mode_parse_error = false;
    auto modeFromString = [this, &mode_parse_error](const std::string & s) -> NavMode {
      if (s == "NORMAL_NAV")      return NavMode::NORMAL_NAV;
      if (s == "SLOW_NAV")        return NavMode::SLOW_NAV;
      if (s == "BLIND_DRIVE")     return NavMode::BLIND_DRIVE;
      if (s == "NAVIGATE_AROUND") return NavMode::NAVIGATE_AROUND;
      if (s == "PUSH_THROUGH")    return NavMode::PUSH_THROUGH;
      // Silently defaulting here once cost us a zone that never triggered.
      RCLCPP_ERROR(get_logger(),
                   "Unknown zone mode '%s'. Valid: NORMAL_NAV, SLOW_NAV, "
                   "BLIND_DRIVE, NAVIGATE_AROUND, PUSH_THROUGH", s.c_str());
      mode_parse_error = true;
      return NavMode::NORMAL_NAV;
    };

    // std::stod / std::stoi throw on malformed input; a bad waypoints file must
    // fail loudly at startup rather than mid-race.
    bool number_parse_error = false;
    auto toDouble = [this, &number_parse_error](
        const std::string & key, const std::string & v, double & out) {
      try {
        size_t consumed = 0;
        const double parsed = std::stod(v, &consumed);
        if (consumed != v.size()) throw std::invalid_argument("trailing characters");
        out = parsed;
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Bad numeric value for '%s': '%s' (%s)",
                     key.c_str(), v.c_str(), e.what());
        number_parse_error = true;
      }
    };

    // Validate and commit a completed zone entry. Sets zone_commit_error on error.
    bool zone_commit_error = false;
    auto commitZone = [this, &zone_commit_error](Zone & z) {
      if (!z.mode_set) {
        RCLCPP_ERROR(get_logger(),
                     "Zone [%s] is missing required 'mode:' field — "
                     "valid values: NORMAL_NAV, SLOW_NAV, BLIND_DRIVE, "
                     "NAVIGATE_AROUND, PUSH_THROUGH",
                     z.label.empty() ? "(unnamed)" : z.label.c_str());
        zone_commit_error = true;
        return;
      }
      zones_.push_back(z);
    };

    std::string line;
    while (std::getline(file, line)) {
      std::string tl = trim(line);
      if (tl.empty()) continue;
      // Indentation of the first non-space character. Anything at column 0 is a
      // top-level key and therefore ends whichever block we were reading;
      // without this an unrelated trailing block's x/y/radius would be absorbed
      // into finish_line.
      const size_t first = line.find_first_not_of(" \t");
      const size_t indent = (first == std::string::npos) ? 0 : first;

      if (indent == 0 && tl == "zones:") {
        in_zones_block  = true;
        in_finish_block = false;
        continue;
      }
      if (indent == 0 && tl.rfind("finish_line:", 0) == 0) {
        if (in_zone_entry) {           // commit any open zone entry
          commitZone(current_zone);
          in_zone_entry = false;
        }
        in_zones_block  = false;
        in_finish_block = true;
        continue;
      }
      // total_laps in the waypoints file wins over the ROS parameter: the
      // waypoints file is the course description and the two must agree.
      if (indent == 0 && tl.rfind("total_laps:", 0) == 0) {
        if (in_zone_entry) { commitZone(current_zone); in_zone_entry = false; }
        in_zones_block  = false;
        in_finish_block = false;
        double laps = 0.0;
        toDouble("total_laps", trim(tl.substr(std::string("total_laps:").size())), laps);
        // Reject fractional values — total_laps must be a whole number.
        if (laps != std::floor(laps)) {
          RCLCPP_ERROR(get_logger(), "total_laps must be an integer, got %g", laps);
          number_parse_error = true;
        }
        const int file_laps = static_cast<int>(laps);
        // The same lower bound that the constructor enforces for the param value.
        if (file_laps < 1) {
          RCLCPP_ERROR(get_logger(), "total_laps must be >= 1 (got %d from waypoints file)",
                       file_laps);
          number_parse_error = true;
        }
        if (file_laps != total_laps_) {
          RCLCPP_INFO(get_logger(), "total_laps: %d (from waypoints file, overriding param %d)",
                      file_laps, total_laps_);
        }
        total_laps_ = file_laps;
        continue;
      }

      // Any other top-level key ends the current block.
      if (indent == 0 && tl.rfind("- ", 0) != 0) {
        if (in_zone_entry) { commitZone(current_zone); in_zone_entry = false; }
        in_zones_block  = false;
        in_finish_block = false;
        continue;
      }

      // Zone list entry start: "- label: ..."
      if (in_zones_block && tl.rfind("- ", 0) == 0) {
        if (in_zone_entry) {
          commitZone(current_zone);  // commit previous zone
        }
        current_zone  = Zone{};
        in_zone_entry = true;
        tl = trim(tl.substr(2));  // strip "- " and parse the inline field
      }

      const size_t colon = tl.find(':');
      if (colon == std::string::npos) continue;

      const std::string key = trim(tl.substr(0, colon));
      std::string       val = trim(tl.substr(colon + 1));
      if (val.empty()) continue;

      // Strip surrounding quotes from string values
      if (val.size() >= 2 &&
          (val.front() == '"' || val.front() == '\'') && val.back() == val.front()) {
        val = val.substr(1, val.size() - 2);
      }

      if (in_zone_entry) {
        if      (key == "label")            current_zone.label = val;
        else if (key == "x")                toDouble(key, val, current_zone.x);
        else if (key == "y")                toDouble(key, val, current_zone.y);
        else if (key == "radius")           toDouble(key, val, current_zone.radius);
        else if (key == "exit_radius")      toDouble(key, val, current_zone.exit_radius);
        else if (key == "mode")             { current_zone.mode = modeFromString(val); current_zone.mode_set = true; }
        else if (key == "inflation_radius") toDouble(key, val, current_zone.inflation_radius);
        else if (key == "ndt_fitness_gate") toDouble(key, val, current_zone.ndt_fitness_gate);
      } else if (in_finish_block) {
        if      (key == "x")      { toDouble(key, val, finish_line_.x);      finish_has_x_      = true; }
        else if (key == "y")      { toDouble(key, val, finish_line_.y);      finish_has_y_      = true; }
        else if (key == "radius") { toDouble(key, val, finish_line_.radius); finish_has_radius_ = true; }
      }
    }

    if (in_zone_entry) {
      commitZone(current_zone);  // commit last open zone
    }

    if (mode_parse_error || number_parse_error || zone_commit_error) {
      RCLCPP_ERROR(get_logger(), "Errors while parsing %s — refusing to start",
                   filepath.c_str());
      return false;
    }
    if (zones_.empty()) {
      RCLCPP_ERROR(get_logger(), "No zones loaded from %s", filepath.c_str());
      return false;
    }
    finish_line_valid_ = (finish_has_x_ && finish_has_y_ && finish_has_radius_);
    if (!finish_line_valid_) {
      RCLCPP_WARN(get_logger(),
                  "finish_line in %s is missing or incomplete (missing:%s%s%s) — "
                  "lap counting DISABLED; the robot will never transition to DONE",
                  filepath.c_str(),
                  finish_has_x_      ? "" : " x",
                  finish_has_y_      ? "" : " y",
                  finish_has_radius_ ? "" : " radius");
    }

    // Validate geometry: radius >= exit_radius removes the hysteresis and makes
    // the zone flip-flop between entered and exited on consecutive ticks.
    for (const auto & z : zones_) {
      if (!(z.radius > 0.0) || !(z.exit_radius > z.radius)) {
        RCLCPP_ERROR(get_logger(),
                     "Zone [%s]: need 0 < radius (%.2f) < exit_radius (%.2f)",
                     z.label.c_str(), z.radius, z.exit_radius);
        return false;
      }
    }

    for (const auto & z : zones_) {
      RCLCPP_INFO(get_logger(),
                  "  Zone [%s]: (%.1f, %.1f) r=%.1f exit_r=%.1f mode=%s",
                  z.label.c_str(), z.x, z.y, z.radius, z.exit_radius,
                  modeToString(z.mode).c_str());
    }
    return true;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<ZoneNavManagerNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("zone_nav_manager"), "Fatal: %s", e.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
