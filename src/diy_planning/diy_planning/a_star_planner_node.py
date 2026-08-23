#!/usr/bin/env python3
"""
A* path planner for the DIY Challenge robot.

Consumes:
    /map
    /goal_pose
    TF map -> base_frame

Publishes:
    /a_star/path
    /a_star/visited_map

The planner includes obstacle inflation so the robot does not plan paths
too close to walls or obstacles.

Simulation:
    base_frame := base_footprint

Hardware:
    base_frame := base_link
"""

import math
from queue import PriorityQueue

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformListener


class GraphNode:

    def __init__(self, x, y, cost=0.0, heuristic=0.0, prev=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.prev = prev

    def __lt__(self, other):
        return (
            self.cost + self.heuristic
            < other.cost + other.heuristic
        )

    def __eq__(self, other):
        return (
            self.x == other.x
            and self.y == other.y
        )

    def __hash__(self):
        return hash((self.x, self.y))

    def __add__(self, other):
        return GraphNode(
            self.x + other[0],
            self.y + other[1]
        )


class AStarPlanner(Node):

    def __init__(self):

        super().__init__('a_star_node')

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

        self.declare_parameter(
            'map_topic',
            '/map'
        )

        self.declare_parameter(
            'goal_topic',
            '/goal_pose'
        )

        self.declare_parameter(
            'path_topic',
            '/a_star/path'
        )

        self.declare_parameter(
            'base_frame',
            'base_footprint'
        )

        # Minimum distance from obstacles.
        #
        # Example:
        # map resolution = 0.05 m
        # clearance = 0.25 m
        #
        # => approximately 5 cells around obstacles
        self.declare_parameter(
            'robot_clearance',
            0.25
        )

        # Treat unknown map cells (-1) as blocked.
        self.declare_parameter(
            'unknown_is_occupied',
            True
        )

        # Occupancy values >= this number are obstacles.
        self.declare_parameter(
            'occupied_threshold',
            50
        )

        # ============================================================
        # Read parameters
        # ============================================================

        map_topic = self.get_parameter(
            'map_topic'
        ).value

        goal_topic = self.get_parameter(
            'goal_topic'
        ).value

        path_topic = self.get_parameter(
            'path_topic'
        ).value

        self.base_frame = self.get_parameter(
            'base_frame'
        ).value

        self.robot_clearance = float(
            self.get_parameter(
                'robot_clearance'
            ).value
        )

        self.unknown_is_occupied = bool(
            self.get_parameter(
                'unknown_is_occupied'
            ).value
        )

        self.occupied_threshold = int(
            self.get_parameter(
                'occupied_threshold'
            ).value
        )

        # ============================================================
        # QoS
        # ============================================================

        map_qos = QoSProfile(
            depth=10
        )

        map_qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ============================================================
        # ROS interfaces
        # ============================================================

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_callback,
            map_qos
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            goal_topic,
            self.goal_callback,
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            path_topic,
            map_qos
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/a_star/visited_map',
            10
        )

        # ============================================================
        # State
        # ============================================================

        self.map_ = None

        self.visited_map_ = (
            OccupancyGrid()
        )

        self.clearance_cells = 0

        self.get_logger().info(
            f'A* planner ready. '
            f'Robot clearance = '
            f'{self.robot_clearance:.2f} m'
        )

    # ================================================================
    # Map callback
    # ================================================================

    def map_callback(
        self,
        map_msg: OccupancyGrid
    ):

        self.map_ = map_msg

        self.visited_map_.header = (
            map_msg.header
        )

        self.visited_map_.info = (
            map_msg.info
        )

        self.visited_map_.data = (
            [-1]
            * (
                map_msg.info.height
                * map_msg.info.width
            )
        )

        # Convert clearance from meters to cells.
        self.clearance_cells = int(
            math.ceil(
                self.robot_clearance
                / map_msg.info.resolution
            )
        )

        self.get_logger().info(
            f'Map received: '
            f'{map_msg.info.width} x '
            f'{map_msg.info.height}, '
            f'resolution='
            f'{map_msg.info.resolution:.3f} m, '
            f'clearance='
            f'{self.clearance_cells} cells'
        )

    # ================================================================
    # Goal callback
    # ================================================================

    def goal_callback(
        self,
        pose: PoseStamped
    ):

        if self.map_ is None:

            self.get_logger().error(
                'No map received!'
            )

            return

        # Reset visited visualization.
        self.visited_map_.data = (
            [-1]
            * (
                self.map_.info.height
                * self.map_.info.width
            )
        )

        if (
            pose.header.frame_id
            != self.map_.header.frame_id
        ):

            self.get_logger().error(
                f"Goal frame "
                f"'{pose.header.frame_id}' "
                f"does not match map frame "
                f"'{self.map_.header.frame_id}'"
            )

            return

        # ------------------------------------------------------------
        # Get current robot position in map frame
        # ------------------------------------------------------------

        try:

            map_to_base_tf = (
                self.tf_buffer.lookup_transform(
                    self.map_.header.frame_id,
                    self.base_frame,
                    rclpy.time.Time()
                )
            )

        except Exception as exception:

            self.get_logger().error(
                f'Could not transform map -> '
                f'{self.base_frame}: '
                f'{exception}'
            )

            return

        start_pose = Pose()

        start_pose.position.x = (
            map_to_base_tf.transform.translation.x
        )

        start_pose.position.y = (
            map_to_base_tf.transform.translation.y
        )

        start_pose.orientation = (
            map_to_base_tf.transform.rotation
        )

        # ------------------------------------------------------------
        # Plan
        # ------------------------------------------------------------

        path = self.plan(
            start_pose,
            pose.pose
        )

        if path.poses:

            self.get_logger().info(
                f'Shortest collision-safe '
                f'path found with '
                f'{len(path.poses)} poses!'
            )

            self.path_pub.publish(
                path
            )

        else:

            self.get_logger().warn(
                'No safe path found to goal.'
            )

    # ================================================================
    # A*
    # ================================================================

    def plan(
        self,
        start: Pose,
        goal: Pose
    ) -> Path:

        # 4-connected grid.
        explore_directions = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
        ]

        start_node = self.world_to_grid(
            start
        )

        goal_node = self.world_to_grid(
            goal
        )

        # ------------------------------------------------------------
        # Validate start / goal
        # ------------------------------------------------------------

        if not self.pose_on_map(start_node):

            self.get_logger().error(
                'Start is outside the map.'
            )

            return Path()

        if not self.pose_on_map(goal_node):

            self.get_logger().error(
                'Goal is outside the map.'
            )

            return Path()

        if not self.is_safe(start_node):

            self.get_logger().error(
                'Start is too close to '
                'an obstacle.'
            )

            return Path()

        if not self.is_safe(goal_node):

            self.get_logger().error(
                'Goal is too close to '
                'an obstacle.'
            )

            return Path()

        # ------------------------------------------------------------
        # Open set
        # ------------------------------------------------------------

        pending_nodes = (
            PriorityQueue()
        )

        start_node.cost = 0.0

        start_node.heuristic = (
            self.manhattan_distance(
                start_node,
                goal_node
            )
        )

        pending_nodes.put(
            start_node
        )

        # Best known cost to each cell.
        g_score = {
            (
                start_node.x,
                start_node.y
            ): 0.0
        }

        # Prevent repeatedly expanding cells.
        closed_set = set()

        active_node = None
        goal_reached = False

        # ------------------------------------------------------------
        # Search
        # ------------------------------------------------------------

        while (
            not pending_nodes.empty()
            and rclpy.ok()
        ):

            active_node = (
                pending_nodes.get()
            )

            active_key = (
                active_node.x,
                active_node.y
            )

            if active_key in closed_set:
                continue

            closed_set.add(
                active_key
            )

            # Goal found.
            if active_node == goal_node:

                goal_reached = True
                break

            # -----------------------------------------------
            # Expand neighbors
            # -----------------------------------------------

            for (
                direction_x,
                direction_y
            ) in explore_directions:

                new_node = (
                    active_node
                    + (
                        direction_x,
                        direction_y
                    )
                )

                if not self.pose_on_map(
                    new_node
                ):
                    continue

                # Important:
                # checks inflated obstacle clearance.
                if not self.is_safe(
                    new_node
                ):
                    continue

                new_key = (
                    new_node.x,
                    new_node.y
                )

                tentative_cost = (
                    active_node.cost
                    + 1.0
                )

                old_cost = g_score.get(
                    new_key,
                    float('inf')
                )

                if tentative_cost >= old_cost:
                    continue

                g_score[new_key] = (
                    tentative_cost
                )

                new_node.cost = (
                    tentative_cost
                )

                new_node.heuristic = (
                    self.manhattan_distance(
                        new_node,
                        goal_node
                    )
                )

                new_node.prev = (
                    active_node
                )

                pending_nodes.put(
                    new_node
                )

            # -----------------------------------------------
            # Visited visualization
            # -----------------------------------------------

            self.visited_map_.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            self.visited_map_.data[
                self.pose_to_cell(
                    active_node
                )
            ] = 50

            # Publishing every iteration can be expensive.
            # For this small map it is acceptable while debugging.
            self.map_pub.publish(
                self.visited_map_
            )

        # ============================================================
        # Reconstruct path
        # ============================================================

        path = Path()

        if not goal_reached:

            return path

        path.header.frame_id = (
            self.map_.header.frame_id
        )

        path.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        while (
            active_node is not None
            and active_node.prev is not None
            and rclpy.ok()
        ):

            pose = self.grid_to_world(
                active_node
            )

            pose_stamped = (
                PoseStamped()
            )

            pose_stamped.header = (
                path.header
            )

            pose_stamped.pose = pose

            path.poses.append(
                pose_stamped
            )

            active_node = (
                active_node.prev
            )

        path.poses.reverse()

        return path

    # ================================================================
    # Clearance / obstacle inflation
    # ================================================================

    def is_safe(
        self,
        node: GraphNode
    ) -> bool:

        """
        A cell is safe only if every cell inside the robot-clearance
        radius is free.

        This effectively inflates walls / obstacles before planning.
        """

        radius = self.clearance_cells

        for dx in range(
            -radius,
            radius + 1
        ):

            for dy in range(
                -radius,
                radius + 1
            ):

                # Circular inflation instead of square inflation.
                if (
                    dx * dx + dy * dy
                    > radius * radius
                ):
                    continue

                check_node = GraphNode(
                    node.x + dx,
                    node.y + dy
                )

                # Map edge behaves like an obstacle.
                if not self.pose_on_map(
                    check_node
                ):
                    return False

                value = self.map_.data[
                    self.pose_to_cell(
                        check_node
                    )
                ]

                # Unknown.
                if (
                    value < 0
                    and self.unknown_is_occupied
                ):
                    return False

                # Occupied.
                if (
                    value
                    >= self.occupied_threshold
                ):
                    return False

        return True

    # ================================================================
    # Heuristic
    # ================================================================

    def manhattan_distance(
        self,
        node: GraphNode,
        goal_node: GraphNode
    ):

        return (
            abs(node.x - goal_node.x)
            + abs(node.y - goal_node.y)
        )

    # ================================================================
    # Map helpers
    # ================================================================

    def pose_on_map(
        self,
        node: GraphNode
    ):

        return (
            0 <= node.x
            < self.map_.info.width
            and
            0 <= node.y
            < self.map_.info.height
        )

    def pose_to_cell(
        self,
        node: GraphNode
    ):

        return (
            node.y
            * self.map_.info.width
            + node.x
        )

    # ================================================================
    # World <-> grid
    # ================================================================

    def world_to_grid(
        self,
        pose: Pose
    ) -> GraphNode:

        grid_x = int(
            math.floor(
                (
                    pose.position.x
                    - self.map_.info.origin.position.x
                )
                / self.map_.info.resolution
            )
        )

        grid_y = int(
            math.floor(
                (
                    pose.position.y
                    - self.map_.info.origin.position.y
                )
                / self.map_.info.resolution
            )
        )

        return GraphNode(
            grid_x,
            grid_y
        )

    def grid_to_world(
        self,
        node: GraphNode
    ) -> Pose:

        pose = Pose()

        # Use center of occupancy-grid cell.
        pose.position.x = (
            (
                node.x + 0.5
            )
            * self.map_.info.resolution
            + self.map_.info.origin.position.x
        )

        pose.position.y = (
            (
                node.y + 0.5
            )
            * self.map_.info.resolution
            + self.map_.info.origin.position.y
        )

        # Valid identity quaternion.
        pose.orientation.w = 1.0

        return pose


def main(args=None):

    rclpy.init(args=args)

    node = AStarPlanner()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()