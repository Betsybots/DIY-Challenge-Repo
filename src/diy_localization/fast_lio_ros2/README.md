# FAST_LIO_Hesai

FAST_LIO_Hesai provides Hesai-adapted FAST-LIO2 support, built for the following platform:

| Component | Detail                    |
| --------- | -------------------------- |
| Compute   | NVIDIA Jetson Orin Nano    |
| OS        | Ubuntu 22.04               |
| ROS       | ROS 2 Humble               |
| LiDAR     | Hesai QT64                 |
| IMU       | ACEINNA IMU                |

## Overview

FAST-LIO2 is a tightly-coupled LiDAR-inertial odometry algorithm. This repository adapts FAST-LIO2 for the Hesai QT64 LiDAR and an ACEINNA IMU through standard point cloud and IMU topics.


## Build

```bash
cd <"your_workspace_package_folder">
colcon build --packages-select fast_lio_ros2 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`--symlink-install` symlinks `config/`, `launch/`, and the `tools/` scripts into `install/` so edits to those files take effect without rebuilding. `-DCMAKE_BUILD_TYPE=Release` is enforced by [CMakeLists.txt](./CMakeLists.txt) even if omitted, but passing it explicitly avoids relying on that default.

## Required Input

Default topics (see `config/qt64.yaml`):

| Data        | Topic           |
| ----------- | --------------- |
| PointCloud2 | `/lidar_points` |
| IMU         | `/imu/data`     |

Required point fields:

```text
x, y, z, intensity, ring, timestamp
```

The point cloud must contain per-point `ring` and `timestamp` fields. The `timestamp` field is used for motion compensation.

## Quick Command

```bash
ros2 launch fast_lio_ros2 lio_localizer.launch.py
```

## Known Notes

* Windows native environment is not supported.
* Jetson Orin Nano running Ubuntu 22.04 + ROS 2 Humble is the target runtime environment.
* The Hesai ROS Driver must publish `/lidar_points` before starting FAST-LIO2.
* `PointCloud2` must contain `ring` and `timestamp` fields.
* QT64 has no built-in IMU; this setup uses an ACEINNA IMU driver publishing to `common.imu_topic` (see `config/qt64.yaml`). Verify the IMU's angular-velocity units (`common.imu_gyr_unit: "deg"` or `"rad"`) before running -- a mismatch causes large stationary drift.

## Issue Reporting

When reporting an issue, please include:

* Compute: Jetson Orin Nano
* ROS version: ROS 2 Humble
* Ubuntu version
* FAST_LIO_Hesai branch / commit
* Hesai ROS Driver version / commit
* Running mode: real-time LiDAR / rosbag replay
* Output of `ros2 topic list`
* Output of `ros2 topic hz /lidar_points` and `ros2 topic hz /imu/data`
* Related logs, screenshots, or rosbag if available
