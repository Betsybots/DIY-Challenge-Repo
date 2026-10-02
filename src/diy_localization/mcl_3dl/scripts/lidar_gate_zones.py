#!/usr/bin/env python3
"""
lidar_gate_zones -- publish whether mcl_3dl should use lidar matching, based on
where the robot is in the map.

Looks up map -> robot_frame from TF at `rate` Hz and publishes std_msgs/Bool on
`mcl_measurement_enabled`: false while the robot is inside any enabled zone of
`zones_file`, true otherwise (and true whenever the TF is unavailable, so the
gate fails open). mcl_3dl only listens when started with
use_measurement_gate:=true.

Zone file format: see config/lidar_gate_zones_obstacle_course.yaml.
"""
import math

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


def dist_point_segment(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    s = 0.0 if L2 == 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + s * dx), py - (ay + s * dy))


def inside_polygon(px, py, pts):
    inside = False
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > py) != (yj > py) and px < (xj - xi) * (py - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


class Zone:
    def __init__(self, cfg):
        self.name = cfg['name']
        self.enabled = bool(cfg.get('enabled', True))
        self.type = cfg.get('type', 'polygon')
        self.points = [(float(x), float(y)) for x, y in cfg['points']]
        self.half_width = float(cfg.get('half_width', 0.5))
        self.heading = cfg.get('heading_deg')
        self.heading_tol = float(cfg.get('heading_tolerance_deg', 180.0))
        if self.type not in ('polygon', 'corridor'):
            raise ValueError(f'zone {self.name}: unknown type {self.type}')
        if self.type == 'polygon' and len(self.points) < 3:
            raise ValueError(f'zone {self.name}: polygon needs >= 3 points')
        if self.type == 'corridor' and len(self.points) < 2:
            raise ValueError(f'zone {self.name}: corridor needs >= 2 points')

    def contains(self, x, y, yaw):
        if not self.enabled:
            return False
        if self.heading is not None:
            d = math.degrees(yaw) - float(self.heading)
            d = (d + 180.0) % 360.0 - 180.0
            if abs(d) > self.heading_tol:
                return False
        if self.type == 'polygon':
            return inside_polygon(x, y, self.points)
        return any(dist_point_segment(x, y, *a, *b) <= self.half_width
                   for a, b in zip(self.points[:-1], self.points[1:]))


def load_zones(path):
    with open(path) as f:
        cfg = yaml.safe_load(f) or {}
    return [Zone(z) for z in cfg.get('zones', [])]


class LidarGateZones(Node):
    def __init__(self):
        super().__init__('lidar_gate_zones')
        self.declare_parameter('zones_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('rate', 10.0)
        zones_file = self.get_parameter('zones_file').value
        if not zones_file:
            raise RuntimeError('parameter zones_file is required')
        self.zones = load_zones(zones_file)
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Bool, 'mcl_measurement_enabled', 10)
        self.active = None
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.tick)
        enabled = [z.name for z in self.zones if z.enabled]
        self.get_logger().info(f'loaded {len(self.zones)} zones from {zones_file}; enabled: {enabled}')

    def tick(self):
        active = []
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, Time())
            t, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            active = [z.name for z in self.zones if z.contains(t.x, t.y, yaw)]
        except TransformException:
            pass
        if active != self.active:
            if active:
                self.get_logger().info(f'in zone {active}: lidar matching OFF (odometry only)')
            elif self.active is not None:
                self.get_logger().info('left gate zones: lidar matching ON')
            self.active = active
        self.pub.publish(Bool(data=not active))


def main():
    rclpy.init()
    node = LidarGateZones()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
