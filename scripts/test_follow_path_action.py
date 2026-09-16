#!/usr/bin/env python3
"""
test_follow_path_action.py — send a manual FollowPath action goal to
controller_server, to test diy_motion_planner::PurePursuitController
in isolation, without needing a real planner (a_star_planner_node's own
nav2_core::GlobalPlanner port is still in progress -- see
costmaps_params.yaml's header).

Sends a short, hardcoded straight-line path a fixed distance in front of
wherever the robot currently is (read via TF), so this is safe to run
repeatedly without editing coordinates for your current position.

PREREQUISITES:
  - Real localization already running and converged (map_localizer + EKF) --
    this script does NOT publish any TF itself, unlike the sandbox tests used
    to build this plugin.
  - costmaps.launch.py already running (controller_server + local_costmap),
    with FollowPath's plugin set to diy_motion_planner::PurePursuitController
    (see costmaps_params.yaml).
  - cmd_vel_mux_node in AUTONOMOUS mode, if you want the robot to actually
    move (controller_server's cmd_vel output is remapped to /cmd_vel_nav --
    same topic the mux forwards only in AUTONOMOUS mode):
        ros2 param set /cmd_vel_mux_node mode AUTONOMOUS

Usage:
    ros2 run diy_motion_planner test_follow_path_action.py
    python3 scripts/test_follow_path_action.py --distance 1.0 --frame map
    python3 scripts/test_follow_path_action.py --dry-run   # print path only

SAFETY: this will actually drive the robot if the mux is in AUTONOMOUS mode.
Keep a hand near the e-stop -- there is no obstacle avoidance path here yet
(the local costmap is running and publishing, but this controller doesn't
consume it for live replanning -- see the planner-side work still pending).
"""

import argparse
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


def build_straight_path(x0, y0, yaw0, distance, frame_id, num_points, clock):
    """A straight line of num_points poses, distance meters ahead of
    (x0, y0, yaw0), all sharing yaw0 (no turning -- pure forward test)."""
    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = clock.now().to_msg()

    for i in range(1, num_points + 1):
        step = distance * i / num_points
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = path.header.stamp
        pose.pose.position.x = x0 + step * math.cos(yaw0)
        pose.pose.position.y = y0 + step * math.sin(yaw0)
        pose.pose.orientation.z = math.sin(yaw0 / 2.0)
        pose.pose.orientation.w = math.cos(yaw0 / 2.0)
        path.poses.append(pose)

    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--distance", type=float, default=1.0,
        help="How far ahead (meters) to send the straight-line test path.")
    parser.add_argument(
        "--frame", default="map",
        help="Frame to build/send the path in (must match global_costmap's "
             "global_frame or be TF-connected to it).")
    parser.add_argument(
        "--base-frame", default="base_link",
        help="Robot's base frame, used to read its current pose via TF.")
    parser.add_argument(
        "--num-points", type=int, default=10,
        help="Number of poses along the straight line.")
    parser.add_argument(
        "--controller-id", default="FollowPath",
        help="Must match controller_plugins' entry in costmaps_params.yaml.")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Print the constructed path and exit without sending the action goal.")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("test_follow_path_action")

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    node.get_logger().info(
        f"Waiting for TF {args.frame} -> {args.base_frame} "
        "(requires real localization already running and converged)..."
    )
    transform = None
    deadline = node.get_clock().now().nanoseconds + int(10e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tf_buffer.can_transform(args.frame, args.base_frame, rclpy.time.Time()):
            transform = tf_buffer.lookup_transform(
                args.frame, args.base_frame, rclpy.time.Time())
            break

    if transform is None:
        node.get_logger().fatal(
            f"Never got {args.frame} -> {args.base_frame} TF after 10s -- "
            "is localization actually running and converged?"
        )
        rclpy.shutdown()
        sys.exit(1)

    x0 = transform.transform.translation.x
    y0 = transform.transform.translation.y
    q = transform.transform.rotation
    yaw0 = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    node.get_logger().info(
        f"Current pose: ({x0:.2f}, {y0:.2f}), yaw={yaw0:.2f} rad -- "
        f"building a {args.distance:.1f}m straight path from here."
    )

    path = build_straight_path(
        x0, y0, yaw0, args.distance, args.frame, args.num_points, node.get_clock())

    if args.dry_run:
        for i, pose in enumerate(path.poses):
            node.get_logger().info(
                f"  wp{i+1}: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
            )
        node.get_logger().info("--dry-run: not sending the action goal.")
        rclpy.shutdown()
        return

    client = ActionClient(node, FollowPath, "/follow_path")
    node.get_logger().info("Waiting for /follow_path action server...")
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().fatal(
            "/follow_path action server never appeared -- is controller_server "
            "actually running (costmaps.launch.py)?"
        )
        rclpy.shutdown()
        sys.exit(1)

    goal = FollowPath.Goal()
    goal.path = path
    goal.controller_id = args.controller_id

    node.get_logger().info(
        f"Sending FollowPath goal ({len(path.poses)} poses, controller_id="
        f"'{args.controller_id}')..."
    )
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()

    if not goal_handle or not goal_handle.accepted:
        node.get_logger().error("Goal was rejected.")
        rclpy.shutdown()
        sys.exit(1)

    node.get_logger().info(
        "Goal accepted -- robot should now be moving if cmd_vel_mux_node is "
        "in AUTONOMOUS mode. Watch: ros2 topic echo /cmd_vel_nav, "
        "ros2 topic echo /pd/goal_reached, ros2 run tf2_ros tf2_echo "
        f"{args.frame} {args.base_frame}"
    )

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result()
    node.get_logger().info(f"Action finished with status: {result.status}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
