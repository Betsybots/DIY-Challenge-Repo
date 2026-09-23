#!/usr/bin/env python3
"""Offline pass 1/2 for testing scan_flattener_node against a ROS1 bag whose
cloud is recorded in world frame: reads /lidar/point_cloud + /lidar/odom from
a ROS1 .bag via python3-rosbag, re-centers each cloud into the sensor frame
using the paired odom pose, and dumps everything to a pickle for
bag_replay_bridge.py to replay as ROS2 topics.

python3-rosbag depends on ROS1's rospy, and ROS2 humble's rosgraph_msgs
package (which only defines Clock, not Log) shadows ROS1's on PYTHONPATH
whenever /opt/ros/humble is sourced (e.g. from .bashrc), breaking rospy's
import (`rospy.core` does `from rosgraph_msgs.msg import Log`). rclpy is
not needed in this script at all, so below we strip any /opt/ros entries
from sys.path before importing rosbag, regardless of what the calling
shell has sourced. bag_replay_bridge.py (which needs rclpy/tf2_ros) is run
separately, with ROS2 sourced, and never imports rosbag.

Usage:
  python3 bag_to_local_cloud.py --bag /path/to.bag --out /tmp/replay_data.pkl
"""
import argparse
import pickle
import sys

sys.path = [p for p in sys.path if not p.startswith('/opt/ros')]

import numpy as np
import rosbag


def quat_to_rot_matrix(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--cloud-topic', default='/lidar/point_cloud')
    parser.add_argument('--odom-topic', default='/lidar/odom')
    args = parser.parse_args()

    odom_times = []
    odom_msgs = []
    with rosbag.Bag(args.bag) as bag:
        for _, msg, _ in bag.read_messages(topics=[args.odom_topic]):
            odom_times.append(msg.header.stamp.to_sec())
            odom_msgs.append(msg)
    odom_times = np.array(odom_times)
    print(f'Loaded {len(odom_msgs)} odom messages.')

    def nearest_odom(t):
        idx = int(np.searchsorted(odom_times, t))
        idx = min(max(idx, 0), len(odom_times) - 1)
        if idx > 0 and abs(odom_times[idx - 1] - t) < abs(odom_times[idx] - t):
            idx -= 1
        return odom_msgs[idx]

    frames = []
    with rosbag.Bag(args.bag) as bag:
        for _, msg, _ in bag.read_messages(topics=[args.cloud_topic]):
            odom = nearest_odom(msg.header.stamp.to_sec())
            t = np.array([
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z,
            ])
            q = np.array([
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w,
            ])
            rot = quat_to_rot_matrix(*q)

            pts_world = np.frombuffer(msg.data, dtype=np.float32).reshape(
                -1, msg.point_step // 4)[:, :3].astype(np.float32)
            pts_local = ((pts_world - t) @ rot).astype(np.float32)

            frames.append({
                'stamp_sec': msg.header.stamp.to_sec(),
                'xyz_local': pts_local,
                'xyz_world': pts_world,
                'position': t.astype(np.float32),
                'quaternion': q.astype(np.float32),
            })

    print(f'Converted {len(frames)} cloud frames.')
    with open(args.out, 'wb') as f:
        pickle.dump(frames, f, protocol=pickle.HIGHEST_PROTOCOL)
    print(f'Wrote {args.out}')


if __name__ == '__main__':
    main()
