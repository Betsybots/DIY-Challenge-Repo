#!/usr/bin/env python3
"""
trigger_map_relocalize.py — one-shot /relocalize service caller for map_localizer
══════════════════════════════════════════════════════════════════════════════
map_localizer_node does NOT auto-load a map at startup — it only loads a PCD
(and sets the initial pose guess) in response to a slam_interfaces/srv/Relocalize
service call (see LIO_Localization's src/map_localizer/src/localizer_node.cpp
relocCB()). Without something to make that call, map_localizer would start,
subscribe to its topics, and just never do anything — no error, no output,
ever (same "silently does nothing" failure shape as several other bugs found
this session).

This script makes that call exactly once, waiting for the service to become
available first (map_localizer_node needs a moment to come up), then exits.
It does not stay running — it is not a long-lived node in the pipeline.

Usage (normally launched by diy_localization/launch/localization.launch.py,
not run directly):
    ros2 run diy_localization trigger_map_relocalize.py \
        --ros-args -p pcd_path:=/path/to/refined_map.pcd \
        -p initial_x:=0.0 -p initial_y:=0.0 -p initial_z:=0.0 \
        -p initial_yaw:=0.0 -p initial_pitch:=0.0 -p initial_roll:=0.0
"""

import sys

import rclpy
from rclpy.node import Node
from slam_interfaces.srv import Relocalize


class MapRelocalizeTrigger(Node):
    def __init__(self):
        super().__init__("trigger_map_relocalize")

        self.declare_parameter("pcd_path", "")
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_z", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("initial_pitch", 0.0)
        self.declare_parameter("initial_roll", 0.0)
        self.declare_parameter("service_wait_timeout_sec", 30.0)

        self.pcd_path = self.get_parameter("pcd_path").value
        if not self.pcd_path:
            self.get_logger().fatal(
                "pcd_path parameter is empty — nothing to load. "
                "Set it via the pcd_path launch argument."
            )
            raise SystemExit(1)

        self.client = self.create_client(Relocalize, "relocalize")

    def run(self):
        timeout = self.get_parameter("service_wait_timeout_sec").value
        self.get_logger().info(
            f"Waiting up to {timeout:.0f}s for /relocalize service "
            "(map_localizer_node must be running)..."
        )
        if not self.client.wait_for_service(timeout_sec=timeout):
            self.get_logger().fatal(
                "/relocalize service never became available — is "
                "map_localizer_node running? Map will NOT be loaded; "
                "map_localizer will silently produce no output."
            )
            return False

        req = Relocalize.Request()
        req.pcd_path = self.pcd_path
        req.x = float(self.get_parameter("initial_x").value)
        req.y = float(self.get_parameter("initial_y").value)
        req.z = float(self.get_parameter("initial_z").value)
        req.yaw = float(self.get_parameter("initial_yaw").value)
        req.pitch = float(self.get_parameter("initial_pitch").value)
        req.roll = float(self.get_parameter("initial_roll").value)

        self.get_logger().info(
            f"Calling /relocalize with pcd_path={self.pcd_path} "
            f"initial pose=({req.x:.2f}, {req.y:.2f}, {req.z:.2f}, "
            f"yaw={req.yaw:.2f})"
        )
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        if future.result() is None:
            self.get_logger().fatal("Service call did not complete (timeout or error).")
            return False

        response = future.result()
        if not response.success:
            self.get_logger().fatal(f"/relocalize FAILED: {response.message}")
            return False

        self.get_logger().info(f"/relocalize succeeded: {response.message}")
        return True


def main():
    rclpy.init()
    node = MapRelocalizeTrigger()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
