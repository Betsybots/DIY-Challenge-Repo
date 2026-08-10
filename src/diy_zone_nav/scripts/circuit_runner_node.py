#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────────────────
# circuit_runner_node.py
# Closed-loop race circuit orchestrator for the competition arena.
#
# WHAT THIS NODE DOES:
#   • Loads a YAML file of x/y/yaw waypoints defining one full circuit lap.
#   • Waits for /green_light (std_msgs/Bool, data=true) before starting.
#   • Navigates each waypoint sequentially using NavigateToPose.
#   • For waypoints marked rotate_in_place: true — after Nav2 reaches the XY
#     position, this node takes over and rotates the robot directly via
#     /cmd_vel (pure angular, no MPPI). This is far more reliable in narrow
#     corridors than letting MPPI handle corners.
#   • On lap completion, re-submits for the next lap unless /nav_mode == "DONE".
#   • If Nav2 rejects or aborts a goal, retries after a brief back-off.
#
# WAYPOINT YAML FIELDS:
#   x, y       (required) — goal position in map frame
#   yaw        (optional, default 0.0) — target heading in radians
#   label      (optional) — human-readable name for logging
#   rotate_in_place (optional, default false) — if true, circuit_runner drives
#              the rotation directly via /cmd_vel after Nav2 reaches XY.
#              Use this for all corners. Nav2 only handles straight legs.
#
# PARAMETERS:
#   circuit_waypoints_file  (string, required)
#   retry_delay_s           (double, 3.0)
#   action_server_timeout_s (double, 10.0)
#   rotate_speed_rad_s      (double, 0.5)   — angular speed for in-place rotation
#   rotate_tolerance_rad    (double, 0.05)  — yaw error threshold to stop rotation
#   cmd_vel_topic           (string, /cmd_vel) — topic to publish rotation commands
#
# TOPICS:
#   Subscribed:  /green_light (std_msgs/Bool)
#                /nav_mode   (std_msgs/String)
#                /odom       (nav_msgs/Odometry) — for yaw feedback during rotation
#   Published:   <cmd_vel_topic> (geometry_msgs/Twist) — during in-place rotation only
#   Action used: /navigate_to_pose (nav2_msgs/NavigateToPose)
# ─────────────────────────────────────────────────────────────────────────────

import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String


def _normalize_angle(a: float) -> float:
    """Wrap angle to [-π, π]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CircuitRunnerNode(Node):

    def __init__(self):
        super().__init__("circuit_runner")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("circuit_waypoints_file", "")
        self.declare_parameter("retry_delay_s", 3.0)
        self.declare_parameter("action_server_timeout_s", 10.0)
        self.declare_parameter("rotate_speed_rad_s", 0.5)
        self.declare_parameter("rotate_tolerance_rad", 0.05)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        wp_file = self.get_parameter(
            "circuit_waypoints_file").get_parameter_value().string_value
        if not wp_file:
            raise RuntimeError(
                "circuit_waypoints_file parameter is empty — cannot start.")

        self.retry_delay_s_      = self.get_parameter("retry_delay_s").get_parameter_value().double_value
        self.action_timeout_s_   = self.get_parameter("action_server_timeout_s").get_parameter_value().double_value
        self.rotate_speed_       = self.get_parameter("rotate_speed_rad_s").get_parameter_value().double_value
        self.rotate_tolerance_   = self.get_parameter("rotate_tolerance_rad").get_parameter_value().double_value
        cmd_vel_topic            = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value

        # ── Load waypoints ───────────────────────────────────────────────────
        self.waypoints_ = self._load_waypoints(wp_file)
        self.get_logger().info(
            f"Loaded {len(self.waypoints_)} circuit waypoints from {wp_file}")

        # ── State ────────────────────────────────────────────────────────────
        self.race_started_  = False
        self.nav_mode_      = "INIT"
        self.goal_handle_   = None
        self.wp_index_      = 0       # current waypoint being navigated
        self.current_yaw_   = 0.0    # latest yaw from /odom
        self.rotating_      = False  # true while in-place rotation is active
        self.rotate_target_ = 0.0   # target yaw for current rotation

        # ── Publishers / Subscribers ─────────────────────────────────────────
        self.cmd_vel_pub_ = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Bool,     "/green_light", self._on_green_light, 10)
        self.create_subscription(String,   "/nav_mode",    self._on_nav_mode,    10)
        self.create_subscription(Odometry, "/odom",        self._on_odom,        10)

        # ── Action client ────────────────────────────────────────────────────
        self.action_client_ = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── Rotation timer (runs at 20 Hz, enabled only during rotation) ────
        self.rotate_timer_ = self.create_timer(0.05, self._rotate_tick)
        self.rotate_timer_.cancel()

        self.get_logger().info(
            "CircuitRunner ready — waiting for /green_light")

    # ── Odom ─────────────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        self.current_yaw_ = _yaw_from_quaternion(msg.pose.pose.orientation)

    # ── Race control ─────────────────────────────────────────────────────────

    def _on_green_light(self, msg: Bool):
        if msg.data and not self.race_started_:
            self.race_started_ = True
            self.wp_index_ = 0
            self.get_logger().info("GREEN LIGHT — starting circuit")
            self._navigate_next()

    def _on_nav_mode(self, msg: String):
        prev = self.nav_mode_
        self.nav_mode_ = msg.data
        if prev != "DONE" and self.nav_mode_ == "DONE":
            self.get_logger().info("Nav mode DONE — stopping circuit runner")
            self.rotate_timer_.cancel()
            self._cancel_current_goal()
            self._stop_robot()

    # ── Sequential waypoint navigation ───────────────────────────────────────

    def _navigate_next(self):
        if self.nav_mode_ == "DONE":
            return
        if self.wp_index_ >= len(self.waypoints_):
            # Lap complete — restart from waypoint 0
            self.wp_index_ = 0
            self.get_logger().info("Lap complete — starting next lap")

        wp = self.waypoints_[self.wp_index_]
        self.get_logger().info(
            f"[wp {self.wp_index_}] Navigating to '{wp['label']}' "
            f"({wp['x']:.2f}, {wp['y']:.2f}) yaw={math.degrees(wp['yaw']):.1f}°"
            f"{' [rotate_in_place]' if wp['rotate_in_place'] else ''}")

        if not self.action_client_.wait_for_server(timeout_sec=self.action_timeout_s_):
            self.get_logger().error("NavigateToPose server not available — retrying")
            self.create_timer(self.retry_delay_s_, self._retry_navigate)
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = wp["x"]
        goal.pose.pose.position.y = wp["y"]
        goal.pose.pose.position.z = 0.0

        if wp["rotate_in_place"]:
            # Tell Nav2 to reach XY only — use current yaw so Nav2 doesn't
            # try to rotate (yaw_goal_tolerance in params handles acceptance).
            # We will handle the rotation ourselves after Nav2 succeeds.
            cur_yaw = self.current_yaw_
            goal.pose.pose.orientation.z = math.sin(cur_yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(cur_yaw / 2.0)
        else:
            yaw = wp["yaw"]
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        future = self.action_client_.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _retry_navigate(self):
        self._navigate_next()

    def _on_goal_response(self, future):
        self.goal_handle_ = future.result()
        if not self.goal_handle_.accepted:
            self.get_logger().error("NavigateToPose REJECTED — retrying")
            self.goal_handle_ = None
            self.create_timer(self.retry_delay_s_, self._retry_navigate)
            return
        self.goal_handle_.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        self.goal_handle_ = None
        result = future.result()
        status = result.status
        wp = self.waypoints_[self.wp_index_]

        if status == GoalStatus.STATUS_SUCCEEDED:
            if wp["rotate_in_place"]:
                # XY reached — now rotate in place to target yaw
                self.get_logger().info(
                    f"XY reached for '{wp['label']}' — starting in-place rotation "
                    f"to {math.degrees(wp['yaw']):.1f}°")
                self._start_rotation(wp["yaw"])
            else:
                self._advance_waypoint()
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f"NavigateToPose CANCELED at '{wp['label']}'")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(
                f"NavigateToPose ABORTED at '{wp['label']}' — retrying in {self.retry_delay_s_:.1f}s")
            self.create_timer(self.retry_delay_s_, self._retry_navigate)
        else:
            self.get_logger().warn(f"NavigateToPose status {status} — advancing anyway")
            self._advance_waypoint()

    def _advance_waypoint(self):
        self.wp_index_ += 1
        self._navigate_next()

    # ── In-place rotation ────────────────────────────────────────────────────

    def _start_rotation(self, target_yaw: float):
        self.rotate_target_ = target_yaw
        self.rotating_ = True
        self.rotate_timer_.reset()

    def _rotate_tick(self):
        if not self.rotating_:
            return

        error = _normalize_angle(self.rotate_target_ - self.current_yaw_)

        if abs(error) < self.rotate_tolerance_:
            # Done — stop robot and advance
            self.rotating_ = False
            self.rotate_timer_.cancel()
            self._stop_robot()
            self.get_logger().info(
                f"Rotation complete — yaw error {math.degrees(error):.1f}°")
            self._advance_waypoint()
            return

        cmd = Twist()
        # Scale speed down as we approach target for smooth stop
        speed = self.rotate_speed_ if abs(error) > 0.3 else self.rotate_speed_ * 0.4
        cmd.angular.z = math.copysign(speed, error)
        self.cmd_vel_pub_.publish(cmd)

    def _stop_robot(self):
        self.cmd_vel_pub_.publish(Twist())

    # ── Goal cancellation ────────────────────────────────────────────────────

    def _cancel_current_goal(self):
        if self.goal_handle_ is None:
            return
        self.get_logger().info("Cancelling in-flight NavigateToPose goal")
        self.goal_handle_.cancel_goal_async()
        self.goal_handle_ = None

    # ── Waypoint loader ───────────────────────────────────────────────────────

    def _load_waypoints(self, filepath: str) -> list:
        if not os.path.isfile(filepath):
            raise RuntimeError(f"Waypoints file not found: {filepath}")
        with open(filepath, "r") as f:
            data = yaml.safe_load(f)
        if "waypoints" not in data or not isinstance(data["waypoints"], list):
            raise RuntimeError(f"Expected top-level 'waypoints' list in {filepath}")
        wps = []
        for i, wp in enumerate(data["waypoints"]):
            if "x" not in wp or "y" not in wp:
                raise RuntimeError(f"Waypoint {i} missing 'x' or 'y'")
            wps.append({
                "x":               float(wp["x"]),
                "y":               float(wp["y"]),
                "yaw":             float(wp.get("yaw", 0.0)),
                "label":           str(wp.get("label", f"wp{i}")),
                "rotate_in_place": bool(wp.get("rotate_in_place", False)),
            })
        if not wps:
            raise RuntimeError(f"No waypoints found in {filepath}")
        return wps


# ── main ─────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CircuitRunnerNode()
        rclpy.spin(node)
    except RuntimeError as exc:
        if node:
            node.get_logger().fatal(f"circuit_runner startup failed: {exc}")
        else:
            print(f"[circuit_runner] FATAL: {exc}")
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
