# zed_apriltag

ROS2 **Humble** node that subscribes to images from the **ZED ROS2 wrapper**
(launched separately) and detects **AprilTags**, publishing each tag's ID and
its meaning from a YAML label map, plus TF transforms.

## What it does

- Subscribes to the ZED wrapper's rectified left image and `camera_info`.
- Detects AprilTags using OpenCV's ArUco AprilTag detector (default `tag36h11`).
- Looks up each tag ID in `config/tag_labels.yaml` (ID0 = "RampDetection", ...).
- Estimates each tag's 6-DoF pose with `solvePnP` from the camera_info intrinsics.
- Publishes:
  - `apriltag/tag_info` (`zed_apriltag/TagInfo`: header, `id`, `label`) — one per detected tag
  - `apriltag/detections` (`apriltag_msgs/AprilTagDetectionArray`)
  - `apriltag/image` (`sensor_msgs/Image`, annotated, optional)
  - TF `<image frame>` -> `tag_<id>`

## Prerequisites (on the Jetson Orin)

1. **ROS2 Humble** installed.
2. The **ZED ROS2 wrapper** running in another terminal:

   ```bash
   ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
   ```

3. Dependencies:

   ```bash
   sudo apt install ros-humble-apriltag-msgs ros-humble-cv-bridge \
     ros-humble-image-transport ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
   ```

## Build

Place this package in a colcon workspace (`~/ros2_ws/src/zed_apriltag`) and:

```bash
cd ~/ros2_ws
colcon build --packages-select zed_apriltag
source install/setup.bash
```

## Run

```bash
ros2 launch zed_apriltag zed_apriltag.launch.py tag_size:=0.16 tag_family:=tag36h11
```

### Parameters

| Parameter             | Default                             | Description                          |
|-----------------------|-------------------------------------|--------------------------------------|
| `tag_size`            | `0.16`                              | Tag edge length in meters            |
| `tag_family`          | `tag36h11`                          | `tag16h5`/`tag25h9`/`tag36h10`/`tag36h11` |
| `image_topic`         | `/zed/zed_node/rgb/color/rect/image` | Rectified image from the ZED wrapper |
| `camera_info_topic`   | `/zed/zed_node/rgb/camera_info`     | Intrinsics for pose estimation       |
| `tag_labels_file`     | `config/tag_labels.yaml` (installed) | YAML mapping tag IDs to labels       |
| `publish_debug_image` | `true`                              | Publish annotated image              |

### Tag labels

Edit `config/tag_labels.yaml` to give each ID a meaning (IDs without an entry
are published with label `UNKNOWN`):

```yaml
tag_labels:
  ID0: "RampDetection"
  ID1: "Narrow Path"
  ID2: "TBD"
```

## Inspect

```bash
ros2 topic echo /apriltag/tag_info
ros2 topic echo /apriltag/detections
ros2 run tf2_tools view_frames
```
