#!/usr/bin/env python3
"""
verify_imu_gyr_unit.py — On-robot spin test to resolve the imu_gyr_unit
question for src/diy_localization/config/fast_lio_hesai_qt64.yaml.

WHY THIS EXISTS
───────────────
fast_lio_hesai_qt64.yaml has `common.imu_gyr_unit: "deg"`. But
imu_can_interface's driver code (imu_node.cpp) applies a DEG2RAD conversion
before publishing /imu/data — so by REP-103 convention, angular_velocity
should already be in rad/s, which would make "deg" wrong (causes ~57x drift
in FAST-LIO2's gyro integration). This can't be resolved by reading code
alone with confidence — it needs one real measurement against a physically
known rotation.

The 57.3x gap between deg/s and rad/s interpretations is so large that even
a rough, hand-driven test unambiguously tells them apart — no lab equipment
needed.

WHICH AXIS TO WATCH
────────────────────
On /imu/data, angular_velocity.z is real-world YAW rate (rotation about the
vehicle's vertical axis) — confirmed from imu_node.cpp's own mount-correction
comments: accel_z_ = raw_y (so Z ends up "up"), and gyro_z_ = pitch_rate (the
chip-native channel for rotation about that same up-pointing axis). This
script integrates angular_velocity.z.

TEST PROCEDURE
──────────────
  1. Mark two lines on the floor at a known, precisely perpendicular angle
     (90° is easiest — check square with a carpenter's square, or use two
     walls that are already perpendicular). Align the robot chassis with
     line 1.
  2. Run this script:
       python3 scripts/verify_imu_gyr_unit.py
     It first measures the stationary gyro bias for a few seconds — do NOT
     move the robot yet, wait for the "Bias captured" message.
  3. After bias capture, the script starts integrating live. Rotate the
     robot IN PLACE (joystick spin) from line 1 to line 2 (90°), then stop.
     Any speed/pausing is fine — this integrates the whole motion regardless
     of profile.
  4. Press Ctrl+C once stopped. The script prints the total integrated
     angle, ASSUMING the raw data is already rad/s.

INTERPRETING THE RESULT
────────────────────────
  Compare the printed "Integrated angle (assuming rad/s)" to the actual
  angle you rotated (90° in the procedure above):

    ~90° (say, 60-120° given a rough hand-driven turn)
        → the raw data IS rad/s. Set imu_gyr_unit: "rad" in
          fast_lio_hesai_qt64.yaml (the "deg" setting is wrong).

    ~90° × 57.3 ≈ 5157° (thousands of degrees — many multiples of a full
    360° circle for a single 90° physical turn)
        → the raw data is actually deg/s. Keep imu_gyr_unit: "deg" — the
          current setting is correct after all.

  These two outcomes are not close to each other — you do not need a
  precise 90° turn to tell them apart confidently.

Usage:
  python3 scripts/verify_imu_gyr_unit.py [--topic /imu/data] [--bias-duration 3.0]
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class GyrUnitVerifier(Node):
    def __init__(self, topic, bias_duration):
        super().__init__('verify_imu_gyr_unit')
        self.bias_duration = bias_duration
        self.bias_samples = []
        self.bias = 0.0
        self.bias_captured = False
        self.bias_start_wall = time.monotonic()

        self.integrated_rad = 0.0
        self.last_stamp = None
        self.sample_count = 0

        self.sub = self.create_subscription(Imu, topic, self.on_imu, 50)
        self.get_logger().info(
            f"Listening on {topic}. Keep the robot COMPLETELY STILL for "
            f"{bias_duration:.1f}s to capture stationary gyro bias...")

    def on_imu(self, msg: Imu):
        gz = msg.angular_velocity.z
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if not self.bias_captured:
            self.bias_samples.append(gz)
            elapsed = time.monotonic() - self.bias_start_wall
            if elapsed >= self.bias_duration:
                self.bias = sum(self.bias_samples) / len(self.bias_samples)
                self.bias_captured = True
                self.last_stamp = stamp
                self.get_logger().info(
                    f"Bias captured: {self.bias:.6f} (raw units)/s over "
                    f"{len(self.bias_samples)} samples. "
                    f"NOW ROTATE THE ROBOT 90 DEGREES, then Ctrl+C when done.")
            return

        if self.last_stamp is None:
            self.last_stamp = stamp
            return

        dt = stamp - self.last_stamp
        self.last_stamp = stamp
        if dt <= 0 or dt > 1.0:
            # Skip bogus/huge gaps (e.g. first sample after bias capture,
            # or a topic dropout) rather than corrupting the integral.
            return

        self.integrated_rad += (gz - self.bias) * dt
        self.sample_count += 1

        if self.sample_count % 25 == 0:
            deg_if_rad = math.degrees(self.integrated_rad)
            print(f"\r  Integrated so far (assuming rad/s): "
                  f"{deg_if_rad:+9.2f} deg   ", end="", flush=True)

    def print_final(self):
        if not self.bias_captured:
            print("\n[ERROR] Stopped before bias capture finished — no result.")
            return
        deg_if_rad = math.degrees(self.integrated_rad)
        print("\n")
        print("=" * 70)
        print("  RESULT")
        print("=" * 70)
        print(f"  Integrated angle (assuming rad/s): {deg_if_rad:+.2f} deg")
        print(f"  Samples integrated: {self.sample_count}")
        print()
        print("  Compare to the actual angle you physically rotated:")
        print("    - Close to that angle (e.g. ~90 deg for a 90 deg turn)")
        print("      -> raw data IS rad/s -> set imu_gyr_unit: \"rad\"")
        print("    - Close to (actual angle x 57.3) (e.g. ~5157 deg for a")
        print("      90 deg turn)")
        print("      -> raw data is deg/s -> keep imu_gyr_unit: \"deg\"")
        print("=" * 70)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/imu/data')
    parser.add_argument('--bias-duration', type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = GyrUnitVerifier(args.topic, args.bias_duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_final()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
