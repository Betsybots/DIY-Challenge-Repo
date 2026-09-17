#!/usr/bin/env python3

"""
Fast A* planner for the DIY Challenge robot.

Consumes:
    /map
    /goal_pose
    TF map -> base_frame

Publishes:
    /a_star/path
    /a_star/visited_map

Features:
    - 8-connected A*
    - heapq priority queue
    - octile heuristic
    - precomputed obstacle inflation
    - O(1) safety lookup during planning
    - diagonal corner-cut prevention
    - visited-map visualization
    - transient-local QoS for map-like topics
"""

import heapq
import math

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_ros import Buffer, TransformListener


class GraphNode:

    def __init__(
        self,
        x,
        y,
        cost=0.0,
        heuristic=0.0,
        prev=None
    ):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.prev = prev

    @property
    def total_cost(self):
        return self.cost + self.heuristic


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
            'visited_map_topic',
            '/a_star/visited_map'
        )

        self.declare_parameter(
            'base_frame',
            'base_footprint'
        )

        self.declare_parameter(
            'robot_clearance',
            0.40
        )

        self.declare_parameter(
            'occupied_threshold',
            50
        )

        self.declare_parameter(
            'unknown_is_occupied',
            True
        )

        self.declare_parameter(
            'visited_publish_interval',
            250
        )

        # ============================================================
        # Read parameters
        # ============================================================

        self.map_topic = self.get_parameter(
            'map_topic'
        ).value

        self.goal_topic = self.get_parameter(
            'goal_topic'
        ).value

        self.path_topic = self.get_parameter(
            'path_topic'
        ).value

        self.visited_map_topic = self.get_parameter(
            'visited_map_topic'
        ).value

        self.base_frame = self.get_parameter(
            'base_frame'
        ).value

        self.robot_clearance = float(
            self.get_parameter(
                'robot_clearance'
            ).value
        )

        self.occupied_threshold = int(
            self.get_parameter(
                'occupied_threshold'
            ).value
        )

        self.unknown_is_occupied = bool(
            self.get_parameter(
                'unknown_is_occupied'
            ).value
        )

        self.visited_publish_interval = int(
            self.get_parameter(
                'visited_publish_interval'
            ).value
        )

        if self.visited_publish_interval < 1:
            self.visited_publish_interval = 1

        # ============================================================
        # QoS
        # ============================================================

        self.map_qos = QoSProfile(
            depth=1
        )

        self.map_qos.reliability = (
            ReliabilityPolicy.RELIABLE
        )

        self.map_qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ============================================================
        # ROS interfaces
        # ============================================================

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            self.map_qos
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            self.path_topic,
            self.map_qos
        )

        self.visited_map_pub = self.create_publisher(
            OccupancyGrid,
            self.visited_map_topic,
            self.map_qos
        )

        # ============================================================
        # State
        # ============================================================

        self.map_ = None
        self.safe_grid = None

        self.visited_map_ = (
            OccupancyGrid()
        )

        self.clearance_cells = 0

        # heapq needs a deterministic tie breaker
        self.heap_counter = 0

        self.get_logger().info(
            'Fast A* planner ready'
        )

        self.get_logger().info(
            f'  robot_clearance = '
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
            f'{map_msg.info.resolution:.3f} m'
        )

        self.get_logger().info(
            f'Inflating obstacles by '
            f'{self.robot_clearance:.2f} m '
            f'({self.clearance_cells} cells)'
        )

        # ------------------------------------------------------------
        # PRECOMPUTE SAFE GRID
        # ------------------------------------------------------------

        self.build_safe_grid()

        # ------------------------------------------------------------
        # Prepare visited map
        # ------------------------------------------------------------

        self.reset_visited_map()

        self.get_logger().info(
            'Inflated safety grid ready'
        )

    # ================================================================
    # Precompute inflated obstacle map
    # ================================================================

    def build_safe_grid(self):

        width = self.map_.info.width
        height = self.map_.info.height

        total_cells = (
            width * height
        )

        # ------------------------------------------------------------
        # Start by assuming every cell is safe.
        # ------------------------------------------------------------

        self.safe_grid = [
            True
        ] * total_cells

        # ------------------------------------------------------------
        # First mark original blocked cells.
        # ------------------------------------------------------------

        blocked_cells = []

        for y in range(height):

            row_offset = y * width

            for x in range(width):

                index = (
                    row_offset + x
                )

                value = (
                    self.map_.data[index]
                )

                blocked = False

                # Unknown
                if value < 0:

                    if self.unknown_is_occupied:
                        blocked = True

                # Occupied
                elif (
                    value
                    >= self.occupied_threshold
                ):

                    blocked = True

                if blocked:

                    self.safe_grid[index] = False

                    blocked_cells.append(
                        (x, y)
                    )

        # ------------------------------------------------------------
        # Precompute circular inflation offsets once.
        # ------------------------------------------------------------

        radius = (
            self.clearance_cells
        )

        inflation_offsets = []

        radius_squared = (
            radius * radius
        )

        for dy in range(
            -radius,
            radius + 1
        ):

            for dx in range(
                -radius,
                radius + 1
            ):

                if (
                    dx * dx
                    + dy * dy
                    <= radius_squared
                ):

                    inflation_offsets.append(
                        (dx, dy)
                    )

        # ------------------------------------------------------------
        # Inflate every blocked cell.
        # ------------------------------------------------------------

        for (
            obstacle_x,
            obstacle_y
        ) in blocked_cells:

            for (
                dx,
                dy
            ) in inflation_offsets:

                nx = (
                    obstacle_x + dx
                )

                ny = (
                    obstacle_y + dy
                )

                if (
                    0 <= nx < width
                    and
                    0 <= ny < height
                ):

                    index = (
                        ny * width
                        + nx
                    )

                    self.safe_grid[index] = False

        # ------------------------------------------------------------
        # Treat borders inside clearance radius as unsafe.
        #
        # This is equivalent to considering outside-map space blocked.
        # ------------------------------------------------------------

        if radius > 0:

            for y in range(height):

                for x in range(width):

                    if (
                        x < radius
                        or
                        y < radius
                        or
                        x >= width - radius
                        or
                        y >= height - radius
                    ):

                        self.safe_grid[
                            y * width + x
                        ] = False

        safe_count = sum(
            1
            for value in self.safe_grid
            if value
        )

        self.get_logger().info(
            f'Safe cells: '
            f'{safe_count}/{total_cells}'
        )

    # ================================================================
    # Goal callback
    # ================================================================

    def goal_callback(
        self,
        goal_msg: PoseStamped
    ):

        if self.map_ is None:

            self.get_logger().error(
                'No map received!'
            )

            return

        if self.safe_grid is None:

            self.get_logger().error(
                'Safety grid has not been generated.'
            )

            return

        # ------------------------------------------------------------
        # Frame check
        # ------------------------------------------------------------

        if (
            goal_msg.header.frame_id
            != self.map_.header.frame_id
        ):

            self.get_logger().error(
                f"Goal frame "
                f"'{goal_msg.header.frame_id}' "
                f"does not match map frame "
                f"'{self.map_.header.frame_id}'"
            )

            return

        # ------------------------------------------------------------
        # Reset visited visualization
        # ------------------------------------------------------------

        self.reset_visited_map()

        # ------------------------------------------------------------
        # Get robot location
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
                f'Could not transform '
                f'{self.map_.header.frame_id} -> '
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

        start_pose.position.z = (
            map_to_base_tf.transform.translation.z
        )

        start_pose.orientation = (
            map_to_base_tf.transform.rotation
        )

        self.get_logger().info(
            f'Planning: '
            f'({start_pose.position.x:.2f}, '
            f'{start_pose.position.y:.2f}) '
            f'-> '
            f'({goal_msg.pose.position.x:.2f}, '
            f'{goal_msg.pose.position.y:.2f})'
        )

        path = self.plan(
            start_pose,
            goal_msg.pose
        )

        if path.poses:

            self.get_logger().info(
                f'Path found: '
                f'{len(path.poses)} poses'
            )

            self.path_pub.publish(
                path
            )

        else:

            self.get_logger().warn(
                'No safe path found.'
            )

    # ================================================================
    # Fast A*
    # ================================================================

    def plan(
        self,
        start: Pose,
        goal: Pose
    ) -> Path:

        start_node = (
            self.world_to_grid(
                start
            )
        )

        goal_node = (
            self.world_to_grid(
                goal
            )
        )

        path = Path()

        path.header.frame_id = (
            self.map_.header.frame_id
        )

        path.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        # ------------------------------------------------------------
        # Validate
        # ------------------------------------------------------------

        if not self.pose_on_map(
            start_node
        ):

            self.get_logger().error(
                'Start outside map'
            )

            return path

        if not self.pose_on_map(
            goal_node
        ):

            self.get_logger().error(
                'Goal outside map'
            )

            return path

        if not self.is_safe(
            start_node
        ):

            self.get_logger().error(
                'Start is inside inflated obstacle region'
            )

            return path

        if not self.is_safe(
            goal_node
        ):

            self.get_logger().error(
                'Goal is inside inflated obstacle region'
            )

            return path

        # ============================================================
        # Directions
        # ============================================================

        directions = [

            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),

            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ]

        # ============================================================
        # heapq open set
        # ============================================================

        open_heap = []

        start_key = (
            start_node.x,
            start_node.y
        )

        goal_key = (
            goal_node.x,
            goal_node.y
        )

        start_h = (
            self.octile_distance(
                start_node,
                goal_node
            )
        )

        # g_score dictionary
        g_score = {
            start_key: 0.0
        }

        # Parent dictionary:
        #
        # child -> parent
        came_from = {}

        self.heap_counter = 0

        heapq.heappush(
            open_heap,
            (
                start_h,
                self.heap_counter,
                start_node.x,
                start_node.y
            )
        )

        closed_set = set()

        expanded_count = 0

        goal_reached = False

        # ============================================================
        # Search
        # ============================================================

        while (
            open_heap
            and rclpy.ok()
        ):

            (
                current_f,
                _,
                current_x,
                current_y
            ) = heapq.heappop(
                open_heap
            )

            current_key = (
                current_x,
                current_y
            )

            if (
                current_key
                in closed_set
            ):

                continue

            closed_set.add(
                current_key
            )

            expanded_count += 1

            # --------------------------------------------------------
            # Visited map
            # --------------------------------------------------------

            current_index = (
                current_y
                * self.map_.info.width
                + current_x
            )

            original_value = (
                self.map_.data[
                    current_index
                ]
            )

            if (
                original_value
                < self.occupied_threshold
            ):

                self.visited_map_.data[
                    current_index
                ] = 50

            if (
                expanded_count
                % self.visited_publish_interval
                == 0
            ):

                self.publish_visited_map()

            # --------------------------------------------------------
            # Goal
            # --------------------------------------------------------

            if (
                current_key
                == goal_key
            ):

                goal_reached = True
                break

            current_g = (
                g_score[current_key]
            )

            # ========================================================
            # Neighbors
            # ========================================================

            for (
                dx,
                dy,
                movement_cost
            ) in directions:

                nx = (
                    current_x + dx
                )

                ny = (
                    current_y + dy
                )

                neighbor = (
                    GraphNode(
                        nx,
                        ny
                    )
                )

                if not self.pose_on_map(
                    neighbor
                ):

                    continue

                # O(1) lookup now
                if not self.is_safe(
                    neighbor
                ):

                    continue

                # ----------------------------------------------------
                # Diagonal corner cutting prevention
                # ----------------------------------------------------

                diagonal = (
                    dx != 0
                    and dy != 0
                )

                if diagonal:

                    horizontal = (
                        GraphNode(
                            current_x + dx,
                            current_y
                        )
                    )

                    vertical = (
                        GraphNode(
                            current_x,
                            current_y + dy
                        )
                    )

                    if (
                        not self.is_safe(
                            horizontal
                        )
                        or
                        not self.is_safe(
                            vertical
                        )
                    ):

                        continue

                neighbor_key = (
                    nx,
                    ny
                )

                tentative_g = (
                    current_g
                    + movement_cost
                )

                existing_g = (
                    g_score.get(
                        neighbor_key,
                        float('inf')
                    )
                )

                if (
                    tentative_g
                    >= existing_g
                ):

                    continue

                # ----------------------------------------------------
                # Better path
                # ----------------------------------------------------

                came_from[
                    neighbor_key
                ] = current_key

                g_score[
                    neighbor_key
                ] = tentative_g

                heuristic = (
                    self.octile_xy(
                        nx,
                        ny,
                        goal_node.x,
                        goal_node.y
                    )
                )

                f_score = (
                    tentative_g
                    + heuristic
                )

                self.heap_counter += 1

                heapq.heappush(
                    open_heap,
                    (
                        f_score,
                        self.heap_counter,
                        nx,
                        ny
                    )
                )

        # ============================================================
        # Final visited visualization
        # ============================================================

        self.publish_visited_map()

        self.get_logger().info(
            f'A* expanded '
            f'{expanded_count} cells'
        )

        if not goal_reached:

            return path

        # ============================================================
        # Reconstruct path
        # ============================================================

        cells = []

        current_key = (
            goal_key
        )

        cells.append(
            current_key
        )

        while (
            current_key
            != start_key
        ):

            if (
                current_key
                not in came_from
            ):

                self.get_logger().error(
                    'Broken A* parent chain'
                )

                return Path()

            current_key = (
                came_from[
                    current_key
                ]
            )

            cells.append(
                current_key
            )

        cells.reverse()

        # ------------------------------------------------------------
        # Convert cells -> Path
        # ------------------------------------------------------------

        for (
            x,
            y
        ) in cells:

            pose = (
                self.grid_to_world_xy(
                    x,
                    y
                )
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

        return path

    # ================================================================
    # FAST safety lookup
    # ================================================================

    def is_safe(
        self,
        node: GraphNode
    ):

        if not self.pose_on_map(
            node
        ):

            return False

        index = (
            node.y
            * self.map_.info.width
            + node.x
        )

        return (
            self.safe_grid[index]
        )

    # ================================================================
    # Visited map
    # ================================================================

    def reset_visited_map(self):

        if self.map_ is None:
            return

        self.visited_map_ = (
            OccupancyGrid()
        )

        self.visited_map_.header = (
            self.map_.header
        )

        self.visited_map_.info = (
            self.map_.info
        )

        self.visited_map_.data = (
            list(self.map_.data)
        )

        self.publish_visited_map()

    def publish_visited_map(self):

        if self.map_ is None:
            return

        self.visited_map_.header.frame_id = (
            self.map_.header.frame_id
        )

        self.visited_map_.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        self.visited_map_pub.publish(
            self.visited_map_
        )

    # ================================================================
    # Heuristic
    # ================================================================

    def octile_distance(
        self,
        node,
        goal
    ):

        return self.octile_xy(
            node.x,
            node.y,
            goal.x,
            goal.y
        )

    def octile_xy(
        self,
        x1,
        y1,
        x2,
        y2
    ):

        dx = abs(
            x1 - x2
        )

        dy = abs(
            y1 - y2
        )

        minimum = min(
            dx,
            dy
        )

        maximum = max(
            dx,
            dy
        )

        return (
            maximum
            + (
                math.sqrt(2.0)
                - 1.0
            )
            * minimum
        )

    # ================================================================
    # Map helpers
    # ================================================================

    def pose_on_map(
        self,
        node
    ):

        return (
            0 <= node.x
            < self.map_.info.width
            and
            0 <= node.y
            < self.map_.info.height
        )

    def world_to_grid(
        self,
        pose: Pose
    ):

        x = int(
            math.floor(
                (
                    pose.position.x
                    - self.map_.info.origin.position.x
                )
                / self.map_.info.resolution
            )
        )

        y = int(
            math.floor(
                (
                    pose.position.y
                    - self.map_.info.origin.position.y
                )
                / self.map_.info.resolution
            )
        )

        return GraphNode(
            x,
            y
        )

    def grid_to_world_xy(
        self,
        x,
        y
    ):

        pose = Pose()

        pose.position.x = (
            (
                x + 0.5
            )
            * self.map_.info.resolution
            + self.map_.info.origin.position.x
        )

        pose.position.y = (
            (
                y + 0.5
            )
            * self.map_.info.resolution
            + self.map_.info.origin.position.y
        )

        pose.position.z = 0.0

        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        return pose


def main(args=None):

    rclpy.init(
        args=args
    )

    node = AStarPlanner()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()