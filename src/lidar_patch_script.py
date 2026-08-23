#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseArray, PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
import ctypes
import struct

class PatchNode(Node):
    def __init__(self):
        super().__init__('utilities_node')
        
        # Adjust input and output topic names as needed
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar_points',  # Change this to your Ignition Gazebo output topic
            self.lidar_listener_callback,
            10)
            
        # Subscribe to the bridged PoseArray
        self.subscription = self.create_subscription(
            PoseArray,
            '/robot_pose_array',
            self.pose_listener_callback,
            10)
            
        self.publisher_ = self.create_publisher(
            PointCloud2,
            '/lidar_points_remapped',        # Point FAST-LIO2 to this topic
            10)
        
        # Publish the isolated single PoseStamped
        self.publisher = self.create_publisher(
            PoseStamped, 
            '/ground_truth_pose', 
            10)
        self.get_logger().info('Patch Node started. Injecting "time" field into PointCloud2...')
        self.get_logger().info('ground Truth Pose also being published...')
        
    def lidar_listener_callback(self, msg):
        # 1. Parse fields from incoming simulation cloud
        # Ignition standard fields: x, y, z, intensity, ring
        field_names = [f.name for f in msg.fields]
        
        # Read the raw points (Generator output)
        points_in = list(pc2.read_points(msg, skip_nans=True))
        
        points_out = []
        for p in points_in:
            # Reconstruct point tuple based on what your simulation emits
            # Assuming incoming data matches: x, y, z, intensity, ring
            x, y, z, intensity, ring = p[0], p[1], p[2], p[3], p[4]
            
            # Inject 0.0 as the fake per-point microsecond timestamp offset
            fake_time = 0.0 
            
            # Pack back into an ordered tuple
            points_out.append((x, y, z, intensity, ring, fake_time))

        # 2. Define the exact layout FAST-LIO2 expects
        # Order matters! x, y, z, intensity, ring, time
        fields_out = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
            PointField(name='time', offset=18, datatype=PointField.FLOAT32, count=1)
        ]

        # 3. Re-serialize into a valid ROS 2 PointCloud2 message
        patched_msg = pc2.create_cloud(msg.header, fields_out, points_out)
        
        # Match incoming sequence variables 
        patched_msg.is_dense = True
        
        # 4. Broadcast the modified cloud
        self.publisher_.publish(patched_msg)

    def pose_listener_callback(self, msg: PoseArray):
        # Pose_V translates into PoseArray. The individual frame names 
        # are stored inside each pose elements' header.frame_id string.
        for pose in msg.poses:    
            gt_msg = PoseStamped()
            gt_msg.header = msg.header # Retains simulation time
            gt_msg.header.frame_id = "map" # Reference frame of ground truth
            gt_msg.pose.position = pose.position
            gt_msg.pose.orientation = pose.orientation
                
            self.publisher.publish(gt_msg)
            break
            
def main(args=None):
    rclpy.init(args=args)
    node = PatchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
