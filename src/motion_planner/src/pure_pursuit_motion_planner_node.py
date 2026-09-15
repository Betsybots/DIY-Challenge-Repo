#!/usr/bin/env python3

"""Pure-pursuit path follower for the DIY Challenge robot."""

from copy import deepcopy
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


class PurePursuitMotionPlanner(Node):

    def __init__(self):
        super().__init__('pure_pursuit_motion_planner_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('path_topic', '/a_star/path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lookahead_distance', 0.2)
        self.declare_parameter('linear_velocity', 0.3)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('rotate_in_place_threshold', 1.0)
        self.declare_parameter('minimum_turning_velocity', 0.05)
        self.declare_parameter('goal_tolerance', 0.15)

        self.path_topic = self.get_parameter('path_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.lookahead_distance = float(
            self.get_parameter('lookahead_distance').value
        )
        self.linear_velocity = float(
            self.get_parameter('linear_velocity').value
        )
        self.max_angular_velocity = float(
            self.get_parameter('max_angular_velocity').value
        )
        self.rotate_in_place_threshold = float(
            self.get_parameter('rotate_in_place_threshold').value
        )
        self.minimum_turning_velocity = float(
            self.get_parameter(
                'minimum_turning_velocity'
            ).value
        )
        self.goal_tolerance = float(
            self.get_parameter('goal_tolerance').value
        )

        self.path_sub = self.create_subscription(
            Path, self.path_topic, self.path_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.next_pose_pub = self.create_publisher(
            PoseStamped, '/pd/next_pose', 10
        )
        self.lookahead_marker_pub = self.create_publisher(
            Marker, '/pd/lookahead_marker', 10
        )
        # See pd_motion_planner_node.py's matching publisher for why this
        # exists — lets an external waypoint sequencer react to "goal
        # reached" without polling; previously this event had no ROS signal
        # at all in either controller.
        self.goal_reached_pub = self.create_publisher(
            Bool, '/pd/goal_reached', 10
        )
        self.global_plan = None
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Pure pursuit ready: {self.path_topic} -> {self.cmd_vel_topic}'
        )

    def path_callback(self, path):
        self.global_plan = path
        self.get_logger().info(
            f'Received path with {len(path.poses)} poses '
            f'in frame "{path.header.frame_id}"'
        )

    def control_loop(self):
        if self.global_plan is None or not self.global_plan.poses:
            return

        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time()
            )
            odom_to_path = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.global_plan.header.frame_id,
                rclpy.time.Time(),
            )
        except Exception as exception:
            self.get_logger().warn(f'Could not transform path or robot: {exception}')
            return

        robot_x = odom_to_base.transform.translation.x
        robot_y = odom_to_base.transform.translation.y
        final_pose = self.transform_pose(
            self.global_plan.poses[-1], odom_to_path
        )

        if math.hypot(final_pose.pose.position.x - robot_x,
                      final_pose.pose.position.y - robot_y) <= self.goal_tolerance:
            self.goal_reached_pub.publish(Bool(data=True))
            self.stop_robot()
            self.global_plan = None
            return

        target = self.select_lookahead_pose(robot_x, robot_y, odom_to_path)
        self.next_pose_pub.publish(target)
        self.publish_lookahead_marker(target)

        rotation = odom_to_base.transform.rotation
        robot_yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        delta_x = target.pose.position.x - robot_x
        delta_y = target.pose.position.y - robot_y
        distance = math.hypot(delta_x, delta_y)
        target_heading = math.atan2(delta_y, delta_x)
        heading_error = self.normalize_angle(
            target_heading - robot_yaw
        )

        if abs(heading_error) > self.rotate_in_place_threshold:
            command = Twist()
            command.angular.z = math.copysign(
                self.max_angular_velocity,
                heading_error,
            )
            self.cmd_pub.publish(command)
            return

        curvature = 2.0 * math.sin(heading_error) / max(distance, 1e-6)
        turning_scale = max(
            self.minimum_turning_velocity / self.linear_velocity,
            1.0 - abs(heading_error) / math.pi,
        )
        linear_velocity = self.linear_velocity * turning_scale

        angular_velocity = max(
            -self.max_angular_velocity,
            min(
                linear_velocity * curvature,
                self.max_angular_velocity,
            ),
        )

        command = Twist()
        command.linear.x = linear_velocity
        command.angular.z = angular_velocity
        self.cmd_pub.publish(command)

    def select_lookahead_pose(self, robot_x, robot_y, transform):
        closest_index = min(
            range(len(self.global_plan.poses)),
            key=lambda index: self.distance_squared(
                self.transform_pose(self.global_plan.poses[index], transform),
                robot_x,
                robot_y,
            ),
        )

        target = self.transform_pose(self.global_plan.poses[-1], transform)
        for pose in self.global_plan.poses[closest_index:]:
            transformed_pose = self.transform_pose(pose, transform)
            if math.hypot(transformed_pose.pose.position.x - robot_x,
                          transformed_pose.pose.position.y - robot_y) >= self.lookahead_distance:
                target = transformed_pose
                break
        return target

    @staticmethod
    def distance_squared(pose, robot_x, robot_y):
        return ((pose.pose.position.x - robot_x) ** 2
                + (pose.pose.position.y - robot_y) ** 2)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def transform_pose(pose, transform):
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        transformed = PoseStamped()
        transformed.header.frame_id = transform.header.frame_id
        transformed.header.stamp = pose.header.stamp
        transformed.pose = deepcopy(pose.pose)
        transformed.pose.position.x = (
            transform.transform.translation.x
            + math.cos(yaw) * pose.pose.position.x
            - math.sin(yaw) * pose.pose.position.y
        )
        transformed.pose.position.y = (
            transform.transform.translation.y
            + math.sin(yaw) * pose.pose.position.x
            + math.cos(yaw) * pose.pose.position.y
        )
        transformed.pose.position.z = (
            transform.transform.translation.z
            + pose.pose.position.z
        )
        return transformed

    def publish_lookahead_marker(self, pose):
        marker = Marker()
        marker.header = pose.header
        marker.ns = 'lookahead'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = pose.pose.position
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        self.lookahead_marker_pub.publish(marker)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitMotionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
