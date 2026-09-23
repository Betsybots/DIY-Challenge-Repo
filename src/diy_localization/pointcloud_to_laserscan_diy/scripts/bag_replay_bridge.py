#!/usr/bin/env python3
"""Pass 2/2: replays the pickle produced by bag_to_local_cloud.py as ROS2
topics for scan_flattener_node to consume -- publishes the re-centered
sensor-frame cloud, optionally the original world-frame cloud for visual
comparison in RViz, and a world -> lidar_link TF so both display coherently
with Fixed Frame = world.

Run this with ROS2 sourced. It never imports rosbag/rospy, so it does not
hit the ROS1/ROS2 rosgraph_msgs conflict that bag_to_local_cloud.py avoids
by running outside a sourced ROS2 shell -- see that script's docstring.

Usage:
  ros2 run pointcloud_to_laserscan_diy bag_replay_bridge.py \
      --ros-args -p data_path:=/tmp/replay_data.pkl
"""
import pickle

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
import tf2_ros


def make_cloud_msg(points_xyz, stamp, frame_id):
    msg = PointCloud2()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.height = 1
    msg.width = points_xyz.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * points_xyz.shape[0]
    msg.is_dense = True
    msg.data = np.ascontiguousarray(points_xyz, dtype=np.float32).tobytes()
    return msg


class BagReplayBridge(Node):
    def __init__(self):
        super().__init__('bag_replay_bridge')

        data_path = self.declare_parameter('data_path', '').value
        self.output_topic = self.declare_parameter('output_topic', '/lidar_points').value
        self.output_frame_id = self.declare_parameter('output_frame_id', 'lidar_link').value
        self.world_frame_id = self.declare_parameter('world_frame_id', 'world').value
        self.publish_world_cloud = self.declare_parameter('publish_world_cloud', True).value
        self.publish_rate_hz = self.declare_parameter('publish_rate_hz', 10.0).value
        self.loop = self.declare_parameter('loop', True).value

        if not data_path:
            raise ValueError(
                'data_path parameter is required (output of bag_to_local_cloud.py), '
                'e.g. --ros-args -p data_path:=/tmp/replay_data.pkl')

        self.get_logger().info(f'Loading {data_path} ...')
        with open(data_path, 'rb') as f:
            self._frames = pickle.load(f)
        self.get_logger().info(f'Loaded {len(self._frames)} cloud frames.')
        self._idx = 0

        self.cloud_pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.world_cloud_pub = None
        if self.publish_world_cloud:
            self.world_cloud_pub = self.create_publisher(
                PointCloud2, self.output_topic + '_world', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        period = 1.0 / max(self.publish_rate_hz, 1e-3)
        self._timer = self.create_timer(period, self._tick)

    def _tick(self):
        if self._idx >= len(self._frames):
            if self.loop:
                self._idx = 0
                self.get_logger().info('Looping bag replay.')
            else:
                self.get_logger().info('Bag replay finished.')
                self._timer.cancel()
                return

        frame = self._frames[self._idx]
        self._idx += 1

        now = self.get_clock().now().to_msg()

        self.cloud_pub.publish(
            make_cloud_msg(frame['xyz_local'], now, self.output_frame_id))
        if self.world_cloud_pub is not None:
            self.world_cloud_pub.publish(
                make_cloud_msg(frame['xyz_world'], now, self.world_frame_id))

        t = frame['position']
        q = frame['quaternion']
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.world_frame_id
        tf_msg.child_frame_id = self.output_frame_id
        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])
        tf_msg.transform.rotation.x = float(q[0])
        tf_msg.transform.rotation.y = float(q[1])
        tf_msg.transform.rotation.z = float(q[2])
        tf_msg.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = BagReplayBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
