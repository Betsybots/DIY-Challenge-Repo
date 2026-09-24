#!/usr/bin/env python3
"""
sensor_covariance_relay — re-stamps sensor covariances before EKF fusion.

WHY THIS EXISTS
───────────────
robot_localization has no per-sensor "weight" parameter: it trusts whatever
covariance each message carries. Two of our inputs report covariances that
would let them completely dominate the fused estimate:

  * /imu/data     — the ACEINNA driver derives its gyro covariance from the
                    datasheet noise density (~8.5e-08 (rad/s)^2). FAST-LIO2
                    already fuses this same IMU internally, so letting it also
                    dominate this EKF double-counts one sensor.
  * /Odometry     — FAST-LIO2 exports its internal iterated-EKF covariance,
                    which is very small once the map is dense. But that filter
                    has no idea whether the scene is static: a moving obstacle
                    in front of a stationary robot shows up as confident
                    ego-motion. The wheel encoders (which say v = 0) need real
                    weight to pull against that.

This node subscribes to both, applies a floor (and optional scale) to the
relevant covariance blocks, and republishes on *_ekf topics that the EKF
consumes instead of the raw ones. Nothing else about the messages is touched,
so FAST-LIO2, RTAB-Map, etc. keep reading the raw topics unchanged.

TUNING KNOBS (all ROS parameters)
─────────────────────────────────
  imu_gyro_cov_floor     minimum variance on each gyro axis, (rad/s)^2.
                         Compare with the wheel yaw-rate covariance in
                         driveStack diffDrive.yaml (twist_covariance_yaw_rate,
                         0.001): equal → equal weight; larger → IMU trusted
                         less. Default 0.004 → wheels get ~4x the weight.
  imu_gyro_cov_scale     multiplier applied BEFORE the floor. Default 1.0.
    lidar_pose_cov_floor   [x, y, yaw] minimum variances (m^2, m^2, rad^2) on
                                                 the FAST-LIO2 pose. Default [0.09, 0.09, 0.04] →
                                                 0.3 m / ~11 deg std. Raise if a moving obstacle still
                         drags the pose; lower if the pose lags the lidar.
  lidar_pose_cov_scale   multiplier applied BEFORE the floor. Default 1.0.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

# Row-major index of the diagonal entries in a 6x6 pose covariance.
_POSE_X, _POSE_Y, _POSE_YAW = 0, 7, 35


def _all_finite(values):
    """True iff every value is a real, finite number (no NaN/Inf).

    robot_localization's EKF has no recovery from a NaN in its state: one
    poisoned update latches NaN into the state/covariance forever, so every
    later cycle keeps failing even once the raw sensor is healthy again
    (e.g. FAST-LIO2 mid gravity-alignment, or a wheel-odom driver dividing
    by a zero dt on its very first sample). Reject the sample instead of
    forwarding it.
    """
    return all(math.isfinite(v) for v in values)


class SensorCovarianceRelay(Node):

    def __init__(self):
        super().__init__('sensor_covariance_relay')

        self.declare_parameter('imu_in', '/imu/data')
        self.declare_parameter('imu_out', '/imu/data_ekf')
        self.declare_parameter('imu_gyro_cov_scale', 1.0)
        self.declare_parameter('imu_gyro_cov_floor', 0.004)

        self.declare_parameter('lidar_odom_in', '/Odometry')
        self.declare_parameter('lidar_odom_out', '/lidar_odom_ekf')
        self.declare_parameter('lidar_pose_cov_scale', 1.0)
        self.declare_parameter('lidar_pose_cov_floor', [0.09, 0.09, 0.04])

        p = self.get_parameter
        self._imu_scale = float(p('imu_gyro_cov_scale').value)
        self._imu_floor = float(p('imu_gyro_cov_floor').value)
        self._lidar_scale = float(p('lidar_pose_cov_scale').value)
        floor = list(p('lidar_pose_cov_floor').value)
        if len(floor) != 3:
            raise ValueError('lidar_pose_cov_floor must be [x, y, yaw]')
        self._lidar_floor_x, self._lidar_floor_y, self._lidar_floor_yaw = (
            float(v) for v in floor)

        imu_in = p('imu_in').value
        imu_out = p('imu_out').value
        lidar_in = p('lidar_odom_in').value
        lidar_out = p('lidar_odom_out').value

        self._imu_pub = self.create_publisher(Imu, imu_out, 10)
        self._lidar_pub = self.create_publisher(Odometry, lidar_out, 10)
        self._imu_sub = self.create_subscription(
            Imu, imu_in, self._on_imu, qos_profile_sensor_data)
        self._lidar_sub = self.create_subscription(
            Odometry, lidar_in, self._on_lidar, 20)

        self.get_logger().info(
            f'IMU {imu_in} -> {imu_out} (gyro cov x{self._imu_scale:g}, '
            f'floor {self._imu_floor:g}); lidar {lidar_in} -> {lidar_out} '
            f'(pose cov x{self._lidar_scale:g}, floor '
            f'[{self._lidar_floor_x:g}, {self._lidar_floor_y:g}, '
            f'{self._lidar_floor_yaw:g}])')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _on_imu(self, msg: Imu):
        av = msg.angular_velocity
        cov = list(msg.angular_velocity_covariance)
        if not _all_finite((av.x, av.y, av.z, *cov)):
            self.get_logger().warn(
                'Dropping non-finite IMU sample (NaN/Inf in angular_velocity '
                'or its covariance) -- not forwarding to the EKF.',
                throttle_duration_sec=5.0)
            return
        # Scale the whole 3x3 block (keeps it positive semi-definite), then
        # floor the diagonal. A leading -1 means "unknown" per REP-145; treat
        # it like zero so the floor takes over.
        if cov[0] < 0.0:
            cov = [0.0] * 9
        cov = [c * self._imu_scale for c in cov]
        for i in (0, 4, 8):
            cov[i] = max(cov[i], self._imu_floor)
        msg.angular_velocity_covariance = cov
        self._imu_pub.publish(msg)

    def _on_lidar(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        cov = list(msg.pose.covariance)
        if not _all_finite((pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w, *cov)):
            self.get_logger().warn(
                'Dropping non-finite lidar odometry sample (NaN/Inf in pose '
                'or its covariance) -- not forwarding to the EKF.',
                throttle_duration_sec=5.0)
            return
        cov = [c * self._lidar_scale for c in cov]
        cov[_POSE_X] = max(cov[_POSE_X], self._lidar_floor_x)
        cov[_POSE_Y] = max(cov[_POSE_Y], self._lidar_floor_y)
        cov[_POSE_YAW] = max(cov[_POSE_YAW], self._lidar_floor_yaw)
        msg.pose.covariance = cov
        self._lidar_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorCovarianceRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # A second SIGINT during teardown (launch forwards one, a terminal
        # Ctrl-C may add another) would otherwise print a traceback.
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
