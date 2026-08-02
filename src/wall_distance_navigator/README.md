# wall_distance_navigator

ROS 2 package that subscribes to LiDAR `sensor_msgs/msg/PointCloud2` data and uses PID control to keep the robot centered between left and right walls.

- Left
- Right

## Published output (control)

Topic: `/cmd_vel` (`geometry_msgs/msg/Twist`)

Twist fields used:

- `linear.x`: forward speed in m/s
- `angular.z`: steering command from PID in rad/s

If either left or right wall points are missing, the node publishes zero velocity for safety.

## Parameters

- `pointcloud_topic` (default: `/lidar_points`)
- `cmd_vel_topic` (default: `/cmd_vel`)
- `side_half_angle_deg` (default: `25.0`)
- `min_valid_range_m` (default: `0.05`)
- `max_valid_range_m` (default: `30.0`)
- `linear_velocity_mps` (default: `0.2`)
- `kp` (default: `1.2`)
- `ki` (default: `0.0`)
- `kd` (default: `0.08`)
- `max_angular_velocity_rps` (default: `1.2`)
- `integral_limit` (default: `2.0`)

## Build and run

From workspace root:

```bash
colcon build --packages-select wall_distance_navigator
source install/setup.bash
ros2 run wall_distance_navigator wall_distance_navigator_node
```
