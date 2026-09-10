# DIY Challenge Repo

ROS 2 Humble · Jetson Orin Nano · Hesai QT64 · FAST-LIO2 · LIO-SAM · Nav2

Autonomous robot software stack for the DIY Robot Challenge 2026.  
Full documentation: **[docs/DIY_Challenge_Robot_Guide.pdf](docs/DIY_Challenge_Robot_Guide.pdf)**  
GPU / CUDA guide: **[docs/Jetson_GPU_Guide.pdf](docs/Jetson_GPU_Guide.pdf)**
Fresh-clone-to-running-localization walkthrough: **[docs/Jetson_Bringup_Guide.pdf](docs/Jetson_Bringup_Guide.pdf)** (source: [docs/jetson_bringup_guide.md](docs/jetson_bringup_guide.md))
Custom nav stack design & zone_nav decoupling: **[docs/Custom_Nav_Stack_Design.pdf](docs/Custom_Nav_Stack_Design.pdf)** (source: [docs/custom_nav_stack_design.md](docs/custom_nav_stack_design.md))

---

## Quick Start

```bash
# 1. Clone with all third-party packages
git clone --recurse-submodules https://github.com/Betsybots/DIY-Challenge-Repo.git
cd DIY-Challenge-Repo

# 2. Build everything (patches + rosdep + colcon)
bash setup.sh jetson          # or: laptop | raspi

# 3. Source the environment
source scripts/env.sh jetson

# 4. Verify hardware is ready
scripts/health_check.sh jetson

# 5. Launch the robot
scripts/run_robot.sh jetson
```

> **Cloned without `--recurse-submodules`?**  
> Run `git submodule update --init --recursive` to populate `third_party_ws/src/`.

---

## Repository Layout

```
DIY-Challenge-Repo/
├── src/                        ← ROS 2 packages (your code)
│   ├── challenge_bringup/      ← Top-level launch, Nav2 config, maps, RViz configs
│   ├── diy_cmd_vel_mux/        ← Priority-based velocity multiplexer (joy / nav / e-stop)
│   ├── diy_estop_controller/   ← STM32 heartbeat watchdog / e-stop
│   ├── diy_localization/       ← FAST-LIO2 + single EKF fusion (wheel+IMU rate+lidar → /odometry/filtered)
│   ├── diy_motor_control_legacy/ ← Differential drive motor controller (wheel odometry)
│   ├── diy_robot_description/  ← URDF / robot_state_publisher
│   ├── diy_sim/                ← Gazebo simulation world and plugins
│   ├── fast_lio_ros2/          ← FAST-LIO2, Hesai QT64 fork (vendored from teammate repo LIO_Localization)
│   └── plan_b/                 ← Autonomous motion plan executor (waypoint sequencer)
├── third_party_ws/src/         ← Git submodules (auto-cloned)
│   ├── LIO-SAM/                ← Offline mapping with loop closure (prior map generation)
│   ├── imu_utils_ros2_humble/  ← IMU Allan variance calibration
│   ├── lidar_imu_calib/        ← Lidar↔IMU extrinsic calibration
│   └── ndt_omp_ros2/           ← NDT-OMP scan matching (map-based localization)
├── scripts/                    ← Operational and test shell scripts
├── profiles/                   ← Device environment files
├── patches/                    ← Build-fix patches applied by setup.sh
├── docs/                       ← PDF guides + source generators
└── setup.sh                    ← One-command first-time setup
```

> `fast_lio_ros2` replaces the old generic `third_party_ws/src/FAST_LIO` submodule
> (hku-mars/FAST_LIO) as the runtime odometry source — see
> [docs/reuse_plan_step1.md](docs/reuse_plan_step1.md) for why.
>
> The Hesai QT64 lidar driver (`hesai_ros_driver`) is NOT vendored anywhere in
> this repo — it's a separate overlay workspace that already lives directly on
> the Jetson (see §5 below and `docs/installation_setup.html`). The
> `third_party_ws/src/livox_ros_driver2` submodule (for Livox-brand lidars —
> different hardware) that used to be here was unused dead weight (zero real
> code/build dependencies anywhere in this repo) and has been removed.


---

## Hardware Stack

| Component | Part |
|-----------|------|
| Compute | NVIDIA Jetson Orin Nano (8 GB, JetPack 6.x) |
| Lidar | Hesai QT64 (64-beam, 10 Hz, Ethernet) |
| Camera / IMU | Stereolabs ZED2i |
| Motor controller | STM32 via micro-ROS (USB serial) |
| Encoders | Dead-wheel encoders (non-driven, slip-free odometry) |

---

## Documentation Guide

The **[DIY_Challenge_Robot_Guide.pdf](docs/DIY_Challenge_Robot_Guide.pdf)** is the primary reference. Use the table below to jump to what you need:

| I want to… | PDF Section |
|---|---|
| Understand the overall system design and node graph | §1 — Repository Structure & Architecture |
| Learn what each ROS 2 package does | §2 — Package Reference |
| Tune SLAM / EKF / Nav2 parameters | §3 — Configuration & Calibration |
| Switch between Jetson / laptop / Raspberry Pi | §4 — Device Profiles |
| **Set up the robot from scratch (new hardware)** | **§5 — First-Time Setup on Robot Hardware** |
| SSH into the Jetson / headless access | §5.6 — Accessing the Jetson Orin Nano |
| Understand GPU usage and lock CPU/GPU clocks | §5.7 — Jetson Orin Nano Performance |
| Build and connect the micro-ROS agent (STM32) | §5.5 — micro-ROS Agent Setup |
| Calibrate the IMU or lidar-IMU extrinsics | §6 — Calibration Procedures |
| Run on competition day (pre-flight, launch, E-stop) | §7 — Competition Day Operations |
| Replay a bag / debug a node on your laptop | §8 — Developer Workflows |
| Look up what a script does and its arguments | §9 — Scripts Reference |
| Fix a common error or sensor issue | §10 — Troubleshooting |
| Add a new sensor, package, or swap the SLAM algorithm | §11 — Extending the Codebase |

### Jetson GPU Guide — [docs/Jetson_GPU_Guide.pdf](docs/Jetson_GPU_Guide.pdf)

| I want to… | GPU Guide Section |
|---|---|
| Understand what the GPU is and when it helps | §1 — What Is the Jetson Nano GPU? |
| Learn CUDA fundamentals (kernels, threads, memory) | §2 — CUDA Programming Basics |
| Use the GPU for inference WITHOUT writing CUDA code | §3 — TensorRT: GPU Inference Without CUDA |
| Add a real-time object detector (YOLO) to the robot | §4 — GPU Inference Node in ROS 2 |
| Speed up lidar point cloud processing on GPU | §5 — GPU-Accelerated Point Cloud Processing |
| Monitor GPU usage and benchmark inference speed | §6 — Verify GPU Usage and Monitor Performance |
| Quick checklist and troubleshooting GPU issues | §7 — Quick-Reference Checklist |

---

## Scripts Overview

> **Full usage guide:** [docs/testing_guide.md](docs/testing_guide.md) — detailed, living reference for every script/launch file/profile: when to run what, on which device, and current known caveats. Update it as issues are found during testing.

| Script | Purpose |
|--------|---------|
| `setup.sh [PROFILE]` | First-time setup: submodules → patches → rosdep → build |
| `scripts/env.sh [PROFILE]` | Source all workspace overlays in the correct order |
| `scripts/run_robot.sh [PROFILE]` | Launch the full robot stack |
| `scripts/health_check.sh [PROFILE]` | Pre-flight hardware and topic verification |
| `scripts/record_bag.sh [PROFILE]` | Record a labelled rosbag |
| `scripts/replay_bag.sh [BAG]` | Replay a bag on laptop with correct clock |
| `scripts/calibrate_imu.sh [PROFILE]` | IMU Allan variance calibration |
| `scripts/calibrate_extrinsics.sh [PROFILE]` | Lidar↔IMU extrinsic calibration |
| `scripts/debug_robot.sh [PROFILE]` | Single-node debug launch |
| `scripts/deploy_bundle.sh` | Push updated configs to the robot over SSH |
| `scripts/test_step1_wheel_odom.sh [--viz rviz\|foxglove] [PROFILE]` | Test wheel odometry + joystick |
| `scripts/test_step2_fused_odom.sh [--viz rviz\|foxglove] [PROFILE]` | Test wheel odom + IMU EKF fusion |
| `scripts/test_step3_fastlio.sh [--viz rviz\|foxglove] [PROFILE]` | Test FAST-LIO2 lidar odometry |
| `scripts/test_step4_motion_plan.sh [--viz rviz\|foxglove] [fastlio\|fused] [PROFILE]` | Autonomous motion plan demo |
| `scripts/generate_course_pgm.py` | Generate a Nav2 map + zone waypoints from the official course diagram |
| `scripts/register_zones_to_map.py` | Re-project diagram-derived zone waypoints onto a real, LIO-SAM-generated map (once available from an arena practice/mapping pass) |

---

## Device Profiles

Profiles in `profiles/` set environment variables for each platform.  
Always source via `scripts/env.sh` (not directly):

```bash
source scripts/env.sh jetson   # Jetson Nano — full hardware stack
source scripts/env.sh laptop   # Development laptop — simulation / bag replay
source scripts/env.sh raspi    # Raspberry Pi — lightweight subset
```

---

## Third-Party Packages

These are managed as **git submodules** — they are not committed inline to keep the repo small. `setup.sh` initialises them and applies two build-fix patches automatically.

| Package | Purpose | Branch |
|---------|---------|--------|
| [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) | Offline mapping with loop closure | `ros2` |
| [imu_utils_ros2_humble](https://github.com/HYD-PG/imu_utils_ros2_humble) | IMU noise calibration | `main` |
| [lidar_imu_calib](https://github.com/KnightSnape/lidar_imu_calib) | Lidar↔IMU extrinsic calibration | `main` |
| [ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2) | NDT-OMP scan matching | `humble` |

Build-fix patches for `lidar_imu_calib` and `ndt_omp_ros2` live in `patches/` and are applied by `setup.sh`.

> The old `FAST_LIO` submodule (hku-mars/FAST_LIO, generic) was removed —
> runtime odometry now comes from `src/fast_lio_ros2/`, a Hesai QT64-specific
> fork vendored directly from the teammate repo `LIO_Localization`.

---

## Prerequisites

- Ubuntu 22.04 (JetPack 6.x on Jetson Orin Nano)
- ROS 2 Humble (`/opt/ros/humble/setup.bash` must exist)
- `colcon-common-extensions`, `python3-rosdep`, `git`
- Jetson Orin Nano: add user to `dialout` group for STM32 USB serial access

See **§5.1** of the PDF guide for the full prerequisites list.

---

## License

MIT — see [LICENSE](LICENSE) if present, otherwise use freely with attribution.
