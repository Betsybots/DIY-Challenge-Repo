# FAST_LIO_Hesai Tools

Helper tools for validating input data and preparing rosbag files before running FAST-LIO2.

---

## tools/check_input.py — Runtime Input Validator

Validates live LiDAR and IMU input (driver running or rosbag playing) before
starting FAST-LIO2.

**Checks:**

| # | Item | Failure means |
|---|------|---------------|
| 1 | `/lidar_points` topic exists | Driver not started |
| 2 | `/lidar_imu` topic exists | Driver not started |
| 3 | PointCloud2 has required fields (`x y z intensity ring timestamp`) | FAST-LIO2 will fail to parse |
| 4 | `timestamp` is per-point and monotonically increasing | Motion undistortion broken |
| 4b | `timestamp_unit` inferred from data (and compared to config if given) | Wrong `timestamp_unit` → undistortion wrong |
| 4c | Frame interval stability / dropped frame detection | Frame loss degrades mapping |
| 5 | `ring` range matches QT64 (0–63) | Wrong `scan_line` config |
| 6 | IMU frequency ≥ 100 Hz | IMU pipeline issue |
| 7 | Gyro magnitude sanity check (deg/s vs rad/s) | Wrong `imu_gyr_unit` config |
| 8 | `frame_id` of both sensors | TF / coordinate frame risk |
| 9 | LiDAR ↔ IMU time-base synchronization | Different clock sources → sync failure |

**Usage:**

```bash
ros2 run fast_lio_ros2 check_input.py

# Standalone (with ROS already running)
python3 tools/check_input.py --lidar_topic /lidar_points --imu_topic /imu/data

# Compare against your configured timestamp_unit
python3 tools/check_input.py --timestamp-unit 0 --gyro-unit deg
```

| Option | Default | Description |
| ------ | ------- | ----------- |
| `--lidar_topic` | `/lidar_points` | Point cloud topic |
| `--imu_topic` | `/imu/data` | IMU topic |
| `--model` | `qt64` | LiDAR model |
| `--timeout` | `8.0` | Seconds to wait for messages |
| `--timestamp-unit` | (none) | Your `preprocess.timestamp_unit` (0–3); enables a mismatch check |
| `--gyro-unit` | `rad` | Unit published by the IMU driver (`rad` or `deg`) |

---

## tools/check_config.py — Static Config Validator

Validates a FAST-LIO2 yaml config **without** running ROS. Customers can run
it on a config file directly to catch the most common misconfigurations.

**Checks:**

| Item | Failure means |
|------|---------------|
| `preprocess.lidar_type` matches QT64 (ROS 2: 3) | Wrong LiDAR enum |
| `preprocess.scan_line` matches QT64 (64) | Wrong line count |
| `preprocess.timestamp_unit` is a valid enum (0–3) | Invalid unit |
| `common.imu_gyr_unit` is `deg` or `rad` | Invalid unit |
| `preprocess.blind` positive and below `det_range` | All points filtered out |
| `mapping.extrinsic_R` is a valid rotation (orthonormal, det ≈ 1) | Bad extrinsic matrix |
| `mapping.extrinsic_T` has 3 elements | Malformed extrinsic |
| `pcd_save.pcd_save_en` vs `map_file_path` writability | PCD won't save |

**Usage:**

```bash
python3 tools/check_config.py --config config/qt64.yaml
```

| Option | Required | Description |
| ------ | -------- | ----------- |
| `--config` | ✓ | Path to the yaml config |

---

## tools/check_map.py — Map Quality Analyzer

Analyzes FAST-LIO2 **output map quality** and suggests likely causes — distinct
from `check_input.py`/`check_config.py`, which check inputs and config.

**Modes:**

```bash
# Offline: analyze a saved PCD
python3 tools/check_map.py --pcd PCD/scans.pcd

# Live: analyze the published map + trajectory
ros2 run fast_lio_ros2 check_map.py --map-topic /Laser_map --odom-topic /Odometry
```

**Metrics → meaning:**

| Metric | High value indicates |
|--------|----------------------|
| Surface thickness (plane-fit residual) | Ghosting / double surfaces / misalignment → check extrinsics, time sync, `imu_gyr_unit` |
| Point count / density | Too sparse → frame drops or `point_filter_num` too large |
| Trajectory Z drift (live) | Insufficient init motion / degeneracy |

Dependencies: `numpy` (required); `open3d` optional (faster PCD loading).
Thickness analysis is numpy-only, so it works without extra packages.

| Option | Default | Description |
| ------ | ------- | ----------- |
| `--pcd` | — | Offline: path to a `.pcd` map |
| `--map-topic` | `/Laser_map` | Live: accumulated map topic |
| `--odom-topic` | `/Odometry` | Live: odometry topic for trajectory |
| `--timeout` | `10` | Live: seconds to wait for the map |

> Live `/Laser_map` requires `publish.map_en` — use `--pcd` for offline analysis
> otherwise.

---

## tools/pcap_to_rosbag/ — PCAP → rosbag Converter

Converts a Hesai QT64 PCAP file to a FAST-LIO2-ready rosbag2 by driving the Hesai ROS 2 Driver in PCAP playback mode and recording the point cloud topic.

> **QT64 has no built-in IMU.** The driver's IMU topic will not carry real
> data when replaying a bare QT64 PCAP, so only `/lidar_points` is recorded.
> Record/merge your external IMU (e.g. ACEINNA) data separately before
> running FAST-LIO2 against the resulting bag.

**Pipeline:**

```
input.pcap
  ↓  Hesai ROS 2 Driver (source_type: 2)
/lidar_points
  ↓  ros2 bag record
output/  (rosbag2 directory)
  ↓  FAST-LIO2 (with external IMU played back alongside)
/Odometry  /path  /cloud_registered  PCD map
```

**Prerequisites:**

- PCAP file parseable by the Hesai ROS 2 Driver
- `correction.csv` and `firetime.csv` calibration files for the LiDAR
- Python 3 with PyYAML: `pip3 install pyyaml`

```bash
bash tools/pcap_to_rosbag/pcap_to_rosbag_ros2.sh \
  --pcap       /data/input.pcap \
  --correction /data/correction.csv \
  --firetime   /data/firetime.csv \
  --output     /data/output \
  --driver-ws  ~/hesai_ros2_ws
```

Output: a rosbag2 directory `/data/output/`.

**All options:**

| Option | Required | Description |
| ------ | -------- | ----------- |
| `--pcap` | ✓ | Path to `.pcap` file |
| `--correction` | ✓ | Path to correction `.csv` |
| `--firetime` | ✓ | Path to firetime `.csv` |
| `--output` | ✓ | Output rosbag2 directory path |
| `--driver-ws` | ✓ | Hesai ROS 2 Driver workspace root |
| `--play-rate` | | PCAP playback speed (default: `1.0`) |
| `--lidar-topic` | | Override lidar topic (default: `/lidar_points`) |
| `--imu-topic` | | Override driver IMU topic name (not recorded; see note above) |

**What the script does internally:**

1. Generates a temporary Hesai ROS 2 Driver config with `source_type: 2` and PCAP paths
2. Backs up the original driver config, applies the temp config
3. Launches the driver in PCAP mode
4. Waits for `/lidar_points` to appear
5. Checks that the point cloud contains `ring` and `timestamp` fields
6. Starts `ros2 bag record` on `/lidar_points`
7. Detects PCAP end (topic silence ≥ 8 s) and stops recording
8. Restores the original driver config
9. Validates the output bag and prints FAST-LIO2 launch commands

**Known limitations:**

```
1. PCAP must be parseable by the Hesai ROS 2 Driver.
2. QT64 has no built-in IMU — the output bag has no usable IMU stream.
3. /lidar_points must contain 'ring' and 'timestamp' fields.
4. timestamp_unit in the FAST-LIO2 config must match the PCAP data.
5. Field names in the driver config (e.g. firetime_file_path) may vary across driver versions —
   verify against your driver's actual config.yaml structure.
```
