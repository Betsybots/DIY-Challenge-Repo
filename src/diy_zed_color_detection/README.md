# diy_zed_color_detection (ROS 2 package)

Detects RED and GREEN objects on a blue background. Thresholds are ROS parameters
loaded from `config/params.yaml`, and they can be changed live.
Targets **ROS 2 Humble** (Ubuntu 22.04, C++17).

## Build (ROS 2 Humble)
    source /opt/ros/humble/setup.bash
    sudo apt install ros-humble-cv-bridge ros-humble-message-filters ros-humble-image-transport   # or use rosdep below
    mkdir -p ~/ros2_ws/src && unzip diy_zed_color_detection.zip -d ~/ros2_ws/src/   # gives ~/ros2_ws/src/diy_zed_color_detection
    cd ~/ros2_ws && rosdep install --from-paths src -y --ignore-src
    colcon build --packages-select diy_zed_color_detection && source install/setup.bash

## Run
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    # with the ZED ROS 2 wrapper running (default topic /zed/zed_node/rgb/color/rect/image)
    ros2 launch diy_zed_color_detection color_detector.launch.py
    # any other camera
    ros2 launch diy_zed_color_detection color_detector.launch.py image_topic:=/camera/image_raw
    # your own thresholds
    ros2 launch diy_zed_color_detection color_detector.launch.py params_file:=/path/params.yaml

## Topics
| Topic | Type | |
|---|---|---|
| `image_topic` parameter (default `/zed/zed_node/rgb/color/rect/image`) | sensor_msgs/Image (sub) | bgr8, rgb8, bgra8 (ZED) and so on |
| `depth_topic` parameter (default `/zed/zed_node/depth/depth_registered`) | sensor_msgs/Image (sub) | 32FC1 metres or 16UC1 mm, used when `use_depth: true` |
| `~/red` | std_msgs/Bool | `true` when a RED object is in the frame, `false` otherwise (one message per image) |
| `~/green` | std_msgs/Bool | same for GREEN |
| `~/debug_image` | sensor_msgs/Image | annotated image (published only when someone subscribes) |
| `~/mask/<NAME>` | sensor_msgs/Image mono8 | per-colour mask, only when `publish_masks: true` |

## Blue background check
An object is only reported when it sits on blue. The node looks at a ring around each red or
green blob, and at least `background.surround_min_ratio` (default 0.5) of that ring must match the
blue range. Blobs that fail are drawn as thin grey boxes labelled "not on blue" in the debug image,
with the blue share around them. Set `background.min_frame_ratio` to also require a minimum
blue share of the whole frame. Set `background.require_surround: false` to turn the check off.

## Distance limit
Only pixels whose ZED depth is between `min_distance_m` and `max_distance_m` (default 0 to 2.0 m)
can be detected. Pixels with no depth (NaN or inf) are ignored too. In the debug image, everything
out of range is darkened. Set `use_depth: false` to detect at any distance with the colour image only.
The ZED depth must be enabled in the wrapper (a depth mode other than NONE).

## No images arriving?
Every 5 s without a frame, the node logs a warning. If nobody publishes the topic, the warning lists
every Image topic that exists. The ZED topic name depends on the wrapper version and camera model:
    ros2 topic list -t | grep sensor_msgs/msg/Image
    ros2 topic info -v /zed/zed_node/rgb/color/rect/image     # shows publisher QoS
    ros2 launch diy_zed_color_detection color_detector.launch.py image_topic:=<topic from the list>
Older wrappers use `/zed/zed_node/rgb/image_rect_color` or a model namespace such as `/zed2i/zed_node/...`.

## Live tuning
    ros2 param set /color_detector RED.sat_range "[120, 255]"
    ros2 param set /color_detector min_area 200.0
    ros2 param dump /color_detector > my_params.yaml   # save what you tuned
Invalid values are rejected with a reason. `colors` can only be set at startup.
To add a colour, add its name to `colors` and give it a block like RED/GREEN.
