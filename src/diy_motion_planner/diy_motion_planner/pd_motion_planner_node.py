#!/usr/bin/env python3

"""
PD path-following motion planner for the DIY Challenge robot.

Consumes the nav_msgs/Path published by the A* planner, obtains the robot pose
from TF, selects a look-ahead point on the path, and publishes velocity commands.

Simulation:
    /a_star/path -> PD -> /cmd_vel -> gz_ros_bridge -> Gazebo

Hardware:
    /a_star/path -> PD -> /cmd_vel_nav -> cmd_vel_mux -> /cmd_vel_safe

TF:
    Simulation:
        odom_frame := odom
        base_frame := base_footprint

    Hardware:
        odom_frame := odom
        base_frame := base_link
"""

import math

import rclpy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from visualization_msgs.msg import Marker

from tf2_ros import Buffer, TransformListener

from tf_transformations import (
    concatenate_matrices,
    inverse_matrix,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_from_matrix,
)


class PDMotionPlanner(Node):

    def __init__(self):
        super().__init__('pd_motion_planner_node')

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter('kp', 2.0)
        self.declare_parameter('kd', 0.1)

        # Look-ahead distance along the A* path
        self.declare_parameter('step_size', 0.2)

        self.declare_parameter(
            'max_linear_velocity',
            0.3
        )

        self.declare_parameter(
            'max_angular_velocity',
            1.0
        )

        self.declare_parameter(
            'path_topic',
            '/a_star/path'
        )

        # Simulation default.
        # Hardware can override with /cmd_vel_nav.
        self.declare_parameter(
            'cmd_vel_topic',
            '/cmd_vel'
        )

        self.declare_parameter(
            'odom_frame',
            'odom'
        )

        self.declare_parameter(
            'base_frame',
            'base_footprint'
        )

        # Stop once robot is inside this radius of FINAL goal.
        self.declare_parameter(
            'goal_tolerance',
            0.15
        )

        # ============================================================
        # Read parameters
        # ============================================================

        self.kp = self.get_parameter(
            'kp'
        ).value

        self.kd = self.get_parameter(
            'kd'
        ).value

        self.step_size = self.get_parameter(
            'step_size'
        ).value

        self.max_linear_velocity = self.get_parameter(
            'max_linear_velocity'
        ).value

        self.max_angular_velocity = self.get_parameter(
            'max_angular_velocity'
        ).value

        self.goal_tolerance = self.get_parameter(
            'goal_tolerance'
        ).value

        self.path_topic = self.get_parameter(
            'path_topic'
        ).value

        self.cmd_vel_topic = self.get_parameter(
            'cmd_vel_topic'
        ).value

        self.odom_frame = self.get_parameter(
            'odom_frame'
        ).value

        self.base_frame = self.get_parameter(
            'base_frame'
        ).value

        # ============================================================
        # ROS interfaces
        # ============================================================

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.next_pose_pub = self.create_publisher(
            PoseStamped,
            '/pd/next_pose',
            10
        )

        self.lookahead_marker_pub = self.create_publisher(
            Marker,
            '/pd/lookahead_marker',
            10
        )

        # 10 Hz controller
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        # ============================================================
        # Controller state
        # ============================================================

        self.global_plan = None

        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0

        self.last_cycle_time = self.get_clock().now()

        self.get_logger().info(
            f'PD controller ready: '
            f'{self.path_topic} -> {self.cmd_vel_topic}'
        )

        self.get_logger().info(
            f'Frames: '
            f'{self.odom_frame} -> {self.base_frame}'
        )

        self.get_logger().info(
            f'Goal tolerance: '
            f'{self.goal_tolerance:.2f} m'
        )

    # ================================================================
    # Path callback
    # ================================================================

    def path_callback(self, path: Path):

        self.global_plan = path

        # Reset derivative state when a new plan arrives.
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0

        self.last_cycle_time = self.get_clock().now()

        self.get_logger().info(
            f'Received path with '
            f'{len(path.poses)} poses '
            f'in frame "{path.header.frame_id}"'
        )

    # ================================================================
    # Main controller
    # ================================================================

    def control_loop(self):

        if self.global_plan is None:
            return

        if not self.global_plan.poses:
            return

        # ------------------------------------------------------------
        # Get current robot pose
        # ------------------------------------------------------------

        try:

            robot_pose_transform = (
                self.tf_buffer.lookup_transform(
                    self.odom_frame,
                    self.base_frame,
                    rclpy.time.Time()
                )
            )

        except Exception as exception:

            self.get_logger().warn(
                f'Could not transform from '
                f'{self.odom_frame} to '
                f'{self.base_frame}: '
                f'{exception}'
            )

            return

        # ------------------------------------------------------------
        # Transform the A* path into the odom frame
        # ------------------------------------------------------------

        target_frame = (
            robot_pose_transform.header.frame_id
        )

        if not self.transform_plan(
            target_frame
        ):

            self.get_logger().error(
                f'Unable to transform plan into '
                f'{target_frame}'
            )

            return

        # ------------------------------------------------------------
        # Build current robot pose
        # ------------------------------------------------------------

        robot_pose = PoseStamped()

        robot_pose.header = (
            robot_pose_transform.header
        )

        robot_pose.pose.position.x = (
            robot_pose_transform.transform.translation.x
        )

        robot_pose.pose.position.y = (
            robot_pose_transform.transform.translation.y
        )

        robot_pose.pose.position.z = (
            robot_pose_transform.transform.translation.z
        )

        robot_pose.pose.orientation = (
            robot_pose_transform.transform.rotation
        )

        # ============================================================
        # FINAL GOAL CHECK
        # ============================================================

        goal_pose = self.global_plan.poses[-1]

        goal_dx = (
            goal_pose.pose.position.x
            - robot_pose.pose.position.x
        )

        goal_dy = (
            goal_pose.pose.position.y
            - robot_pose.pose.position.y
        )

        goal_distance = math.hypot(
            goal_dx,
            goal_dy
        )

        self.get_logger().info(
            f'Distance to goal: '
            f'{goal_distance:.3f} m'
        )

        # ------------------------------------------------------------
        # STOP once robot reaches final goal region
        # ------------------------------------------------------------

        if goal_distance <= self.goal_tolerance:

            self.get_logger().info(
                f'Goal reached! '
                f'Distance = {goal_distance:.3f} m'
            )

            # Explicit zero command.
            self.stop_robot()

            # Disable controller until a NEW path is received.
            self.global_plan = None

            # Reset PD memory.
            self.prev_linear_error = 0.0
            self.prev_angular_error = 0.0

            return

        # ============================================================
        # Select look-ahead point
        # ============================================================

        next_pose = self.get_next_pose(
            robot_pose
        )

        self.next_pose_pub.publish(
            next_pose
        )

        self.publish_lookahead_marker(
            next_pose
        )

        # ============================================================
        # Robot transform matrix
        # ============================================================

        robot_transform = quaternion_matrix([
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z,
            robot_pose.pose.orientation.w,
        ])

        robot_transform[0, 3] = (
            robot_pose.pose.position.x
        )

        robot_transform[1, 3] = (
            robot_pose.pose.position.y
        )

        robot_transform[2, 3] = (
            robot_pose.pose.position.z
        )

        # ============================================================
        # Look-ahead pose transform matrix
        # ============================================================

        next_pose_transform = quaternion_matrix([
            next_pose.pose.orientation.x,
            next_pose.pose.orientation.y,
            next_pose.pose.orientation.z,
            next_pose.pose.orientation.w,
        ])

        next_pose_transform[0, 3] = (
            next_pose.pose.position.x
        )

        next_pose_transform[1, 3] = (
            next_pose.pose.position.y
        )

        next_pose_transform[2, 3] = (
            next_pose.pose.position.z
        )

        # ============================================================
        # Express look-ahead point in robot frame
        #
        # Equivalent to C++:
        #
        # next_pose_robot_tf =
        #     robot_tf.inverse() * next_pose_tf;
        # ============================================================

        next_pose_robot_transform = (
            concatenate_matrices(
                inverse_matrix(
                    robot_transform
                ),
                next_pose_transform
            )
        )

        # ------------------------------------------------------------
        # Errors
        # ------------------------------------------------------------

        # Forward error in robot frame.
        linear_error = (
            next_pose_robot_transform[0, 3]
        )

        # Lateral error in robot frame.
        angular_error = (
            next_pose_robot_transform[1, 3]
        )

        # ============================================================
        # dt
        # ============================================================

        current_time = self.get_clock().now()

        dt = (
            current_time
            - self.last_cycle_time
        ).nanoseconds * 1e-9

        self.last_cycle_time = current_time

        # Protect against zero / tiny dt.
        if dt <= 1e-6:
            dt = 0.1

        # ============================================================
        # PD controller
        # ============================================================

        linear_error_derivative = (
            linear_error
            - self.prev_linear_error
        ) / dt

        angular_error_derivative = (
            angular_error
            - self.prev_angular_error
        ) / dt

        linear_velocity = (
            self.kp * linear_error
            + self.kd * linear_error_derivative
        )

        angular_velocity = (
            self.kp * angular_error
            + self.kd * angular_error_derivative
        )

        # ============================================================
        # Saturation
        # ============================================================

        linear_velocity = max(
            -self.max_linear_velocity,
            min(
                linear_velocity,
                self.max_linear_velocity
            )
        )

        angular_velocity = max(
            -self.max_angular_velocity,
            min(
                angular_velocity,
                self.max_angular_velocity
            )
        )

        # ============================================================
        # Publish velocity command
        # ============================================================

        command = Twist()

        command.linear.x = (
            linear_velocity
        )

        command.angular.z = (
            angular_velocity
        )

        self.cmd_pub.publish(
            command
        )

        self.prev_linear_error = (
            linear_error
        )

        self.prev_angular_error = (
            angular_error
        )

    # ================================================================
    # Look-ahead selection
    # ================================================================

    def get_next_pose(
        self,
        robot_pose: PoseStamped
    ) -> PoseStamped:

        """
        Look-ahead logic based on the C++ implementation.

        1. Find the path point closest to the robot.
        2. Search forward from that path index.
        3. Select the first point farther than step_size.
        4. If no point satisfies that condition, use the final goal.
        """

        # ------------------------------------------------------------
        # Find closest point
        # ------------------------------------------------------------

        closest_index = 0

        minimum_distance_squared = (
            float('inf')
        )

        for index, pose in enumerate(
            self.global_plan.poses
        ):

            dx = (
                pose.pose.position.x
                - robot_pose.pose.position.x
            )

            dy = (
                pose.pose.position.y
                - robot_pose.pose.position.y
            )

            distance_squared = (
                dx * dx
                + dy * dy
            )

            if (
                distance_squared
                < minimum_distance_squared
            ):

                minimum_distance_squared = (
                    distance_squared
                )

                closest_index = index

        # ------------------------------------------------------------
        # Default target = final goal
        # ------------------------------------------------------------

        next_pose = (
            self.global_plan.poses[-1]
        )

        # ------------------------------------------------------------
        # Search forward from closest point
        # ------------------------------------------------------------

        for index in range(
            closest_index,
            len(self.global_plan.poses)
        ):

            pose = (
                self.global_plan.poses[index]
            )

            dx = (
                pose.pose.position.x
                - robot_pose.pose.position.x
            )

            dy = (
                pose.pose.position.y
                - robot_pose.pose.position.y
            )

            distance = math.hypot(
                dx,
                dy
            )

            if distance > self.step_size:

                next_pose = pose
                break

        return next_pose

    # ================================================================
    # Transform complete path
    # ================================================================

    def transform_plan(
        self,
        frame: str
    ) -> bool:

        if (
            self.global_plan.header.frame_id
            == frame
        ):
            return True

        source_frame = (
            self.global_plan.header.frame_id
        )

        try:

            # target_frame <- source_frame
            transform = (
                self.tf_buffer.lookup_transform(
                    frame,
                    source_frame,
                    rclpy.time.Time()
                )
            )

        except Exception as exception:

            self.get_logger().error(
                f"Couldn't transform plan from "
                f"{source_frame} to "
                f"{frame}: "
                f"{exception}"
            )

            return False

        # ------------------------------------------------------------
        # Transform matrix
        # ------------------------------------------------------------

        transform_matrix = quaternion_matrix([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ])

        transform_matrix[0, 3] = (
            transform.transform.translation.x
        )

        transform_matrix[1, 3] = (
            transform.transform.translation.y
        )

        transform_matrix[2, 3] = (
            transform.transform.translation.z
        )

        # ------------------------------------------------------------
        # Transform every path pose
        #
        # target_pose =
        #     target_from_source * source_pose
        # ------------------------------------------------------------

        for pose in self.global_plan.poses:

            pose_matrix = quaternion_matrix([
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ])

            pose_matrix[0, 3] = (
                pose.pose.position.x
            )

            pose_matrix[1, 3] = (
                pose.pose.position.y
            )

            pose_matrix[2, 3] = (
                pose.pose.position.z
            )

            transformed_pose = (
                concatenate_matrices(
                    transform_matrix,
                    pose_matrix
                )
            )

            quaternion = (
                quaternion_from_matrix(
                    transformed_pose
                )
            )

            translation = (
                translation_from_matrix(
                    transformed_pose
                )
            )

            pose.pose.orientation.x = (
                quaternion[0]
            )

            pose.pose.orientation.y = (
                quaternion[1]
            )

            pose.pose.orientation.z = (
                quaternion[2]
            )

            pose.pose.orientation.w = (
                quaternion[3]
            )

            pose.pose.position.x = (
                translation[0]
            )

            pose.pose.position.y = (
                translation[1]
            )

            pose.pose.position.z = (
                translation[2]
            )

            pose.header.frame_id = (
                frame
            )

        self.global_plan.header.frame_id = (
            frame
        )

        return True

    # ================================================================
    # RViz look-ahead marker
    # ================================================================

    def publish_lookahead_marker(
        self,
        next_pose: PoseStamped
    ):

        marker = Marker()

        marker.header.frame_id = (
            next_pose.header.frame_id
        )

        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = 'lookahead'
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position = (
            next_pose.pose.position
        )

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Yellow
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.lookahead_marker_pub.publish(
            marker
        )

    # ================================================================
    # Stop robot
    # ================================================================

    def stop_robot(self):

        stop_command = Twist()

        stop_command.linear.x = 0.0
        stop_command.angular.z = 0.0

        self.cmd_pub.publish(
            stop_command
        )


def main(args=None):

    rclpy.init(args=args)

    node = PDMotionPlanner()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # Safety stop when controller exits.
        node.stop_robot()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()