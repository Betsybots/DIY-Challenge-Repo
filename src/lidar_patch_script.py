#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseArray, PoseStamped
import sensor_msgs_py.point_cloud2 as pc2
import math

class PatchNode(Node):
    def __init__(self):
        super().__init__('utilities_node')
        self.declare_parameter('scan_rate_hz', 10.0)
        self.declare_parameter('use_incoming_point_time', False)
        self.last_cloud_stamp = None
        
        # Adjust input and output topic names as needed
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar_points',  # Change this to your Ignition Gazebo output topic
            self.lidar_listener_callback,
            qos_profile_sensor_data)
            
        # Subscribe to the bridged PoseArray
        self.subscription = self.create_subscription(
            PoseArray,
            '/robot_pose_array',
            self.pose_listener_callback,
            10)
            
        self.publisher_ = self.create_publisher(
            PointCloud2,
            '/lidar_points_remapped',        # Point FAST-LIO2 to this topic
            qos_profile_sensor_data)
        
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
        time_field_index = None
        use_incoming_point_time = self.get_parameter('use_incoming_point_time').value
        if use_incoming_point_time:
            for candidate in ('time', 't'):
                if candidate in field_names:
                    time_field_index = field_names.index(candidate)
                    break
        
        # Keep skip_nans disabled so original index->column timing mapping remains valid.
        points_in = pc2.read_points(msg, skip_nans=False)
        width = max(int(msg.width), 1)
        height = max(int(msg.height), 1)
        columns = width if height > 1 else width
        
        points_out = []
        for index, p in enumerate(points_in):
            # Reconstruct point tuple based on what your simulation emits
            # Assuming incoming data matches: x, y, z, intensity, ring
            x, y, z, intensity, ring = p[0], p[1], p[2], p[3], p[4]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            column_index = index % columns
            
            # FAST-LIO expects a per-point relative time within the scan.
            if time_field_index is not None:
                point_time = float(p[time_field_index])
            elif columns > 1:
                point_time = 0.0
            else:
                point_time = 0.0
            
            # Pack back into the Velodyne point layout FAST-LIO expects.
            points_out.append((x, y, z, intensity, point_time, ring))

        if not points_out:
            return

        # FAST-LIO checks only the final point timestamp to decide whether
        # per-point timing exists. Ensure monotonically increasing positive times.
        # 2. Define the exact layout FAST-LIO2 expects
        # Order matters! x, y, z, intensity, ring, time
        fields_out = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1)
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
    except ExternalShutdownException:
        # Another component requested shutdown; treat as normal termination.
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
