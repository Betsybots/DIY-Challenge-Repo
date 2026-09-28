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
so FAST-LIO2, map_localizer, etc. keep reading the raw topics unchanged.

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

GYRO Z BIAS REMOVAL
───────────────────
robot_localization does not estimate gyro bias, and the odom EKF now takes
its heading only from the IMU yaw rate (config/ekf_fusion.yaml). The ACEINNA
z bias measured on the speed-course bags is about -0.0017 rad/s, i.e. ~23 deg
of heading drift over a 230 s run if left in. Whenever /wheel_odom reports
the robot standing still (|vx| and |vyaw| below stationary_speed_threshold
for at least stationary_min_duration), this node averages angular_velocity.z
into a bias estimate and subtracts it from every forwarded IMU sample.
Offline check (both speed-course runs): wheel vx + gyro heading with this
bias removed closed each ~112 m lap to within 0.7-0.8 m of the start.

  gyro_bias_estimation         enable/disable. Default true.
  wheel_odom_in                topic used for the stationary check.
  stationary_speed_threshold   m/s and rad/s. Default 0.01.
  stationary_min_duration      s the robot must be still before samples
                               count (skips deceleration transients).
                               Default 0.5.
  gyro_bias_window             max samples in the running mean; beyond this
                               it becomes an exponential average, so a slow
                               thermal bias change is still tracked.
                               Default 2000 (~10 s at 200 Hz).

GATED FAST-LIO2 BODY TWIST (lidar_twist_out)
────────────────────────────────────────────
FAST-LIO2 leaves /Odometry's twist empty, so this node differentiates
consecutive FAST-LIO2 poses into a body-frame (vx, vy, vyaw) and publishes
it for the EKF, where it is the primary motion input (its map-matched motion
is locally the most accurate: sharpest stacked-scan walls on both
speed-course runs). Each component is checked against an independent
sensor over the same interval:

  * vx vs wheel forward speed: in long straight parallel-walled sections the
    LiDAR cannot observe along-corridor motion and FAST-LIO2 was confirmed to
    slide backward at up to ~1.5 m/s while the wheels read ~0.4 m/s forward.
  * vyaw vs bias-corrected gyro yaw rate.

A component that disagrees by more than twist_vx_gate / twist_wz_gate is
published with a huge variance, so the EKF effectively ignores it for that
step and follows wheels / gyro instead. The check must be done here rather
than with the EKF's own Mahalanobis gate: that gate compares against the
fused state, which FAST-LIO2 dominates, so a slide that builds up gradually
drags the state with it and never trips. Offline on run-2 this gating
removed the corridor ghost and cut the end-of-lap error from 11.2 m to 3.0 m.

  lidar_twist_cov    [vx, vy, vyaw] variances for accepted components.
                     Default [0.0004, 0.0004, 0.05]: vyaw deliberately weak,
                     the gyro is the main turn-rate source (FAST-LIO2's yaw
                     drifted ~45 deg through a hairpin on a re-timed run-2
                     replay, too gradually for the per-step gate).
  twist_vx_gate      m/s. Default 0.2.
  twist_wz_gate      rad/s. Default 0.3.
  lidar_twist_max_dt skip differentiation across pose gaps longer than this.

WHEEL STALL / SPIN (one-sided vx check)
───────────────────────────────────────
The vx check above assumes FAST-LIO2 is the one that is wrong (corridor
slide: FAST-LIO2 reports motion the wheels do not). The opposite also
happens: on the 2026-09-27 obstacle-course run-2 the robot got stuck while
manoeuvring and the wheels reported ~0.5 m of reverse motion three times in
a row while an independent LiDAR scan match showed it moved 1-3 cm. The EKF
followed the wheels and put the robot ~1.5 m off, which later drew the
northbound tunnel leg on top of the southbound wide-section leg.

So when FAST-LIO2 says the robot is still (|vx| < stall_lidar_speed) while
the wheels say it is moving (|vx| > stall_wheel_speed), FAST-LIO2 is kept
rather than gated. If that persists for stall_min_steps FAST-LIO2 steps
(skips the single-step lag when the robot starts moving), the wheels' vx is
rejected until it ends.

  stall_detection    enable/disable. Default true.
  stall_lidar_speed  m/s. Default 0.05.
  stall_wheel_speed  m/s. Default 0.15.
  stall_min_steps    Default 3 (0.3 s at 10 Hz).

WHEEL ODOMETRY (wheel_odom_out)
───────────────────────────────
/wheel_odom republished with its vx variance floored to wheel_vx_cov_floor.
The driver reports 0.001 at 100 Hz, which would out-vote FAST-LIO2's 10 Hz
twist; the floor (default 0.04) makes the wheels a fallback for forward
speed rather than the main source.
"""

import collections
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

# Row-major index of the diagonal entries in a 6x6 pose covariance.
_POSE_X, _POSE_Y, _POSE_YAW = 0, 7, 35
# Same layout for the 6x6 twist covariance.
_TW_VX, _TW_VY, _TW_VYAW = 0, 7, 35
# Variance used to effectively switch off a gated twist component.
_REJECTED_VAR = 1e6
# Seconds of wheel / gyro history kept for checking FAST-LIO2 intervals
# (FAST-LIO2 publishes up to ~0.5 s after the scan it describes).
_HISTORY_S = 5.0


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


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

        self.declare_parameter('gyro_bias_estimation', True)
        self.declare_parameter('wheel_odom_in', '/wheel_odom')
        self.declare_parameter('stationary_speed_threshold', 0.01)
        self.declare_parameter('stationary_min_duration', 0.5)
        self.declare_parameter('gyro_bias_window', 2000)

        self.declare_parameter('lidar_twist_out', '/lidar_twist_ekf')
        self.declare_parameter('lidar_twist_cov', [0.0004, 0.0004, 0.05])
        self.declare_parameter('lidar_twist_max_dt', 0.5)
        self.declare_parameter('twist_vx_gate', 0.2)
        self.declare_parameter('twist_wz_gate', 0.3)
        self.declare_parameter('stall_detection', True)
        self.declare_parameter('stall_lidar_speed', 0.05)
        self.declare_parameter('stall_wheel_speed', 0.15)
        self.declare_parameter('stall_min_steps', 3)
        self.declare_parameter('wheel_odom_out', '/wheel_odom_ekf')
        self.declare_parameter('wheel_vx_cov_floor', 0.04)

        p = self.get_parameter
        self._imu_scale = float(p('imu_gyro_cov_scale').value)
        self._imu_floor = float(p('imu_gyro_cov_floor').value)
        self._lidar_scale = float(p('lidar_pose_cov_scale').value)
        floor = list(p('lidar_pose_cov_floor').value)
        if len(floor) != 3:
            raise ValueError('lidar_pose_cov_floor must be [x, y, yaw]')
        self._lidar_floor_x, self._lidar_floor_y, self._lidar_floor_yaw = (
            float(v) for v in floor)

        self._bias_enabled = bool(p('gyro_bias_estimation').value)
        self._still_thresh = float(p('stationary_speed_threshold').value)
        self._still_min_dur = float(p('stationary_min_duration').value)
        self._bias_window = max(1, int(p('gyro_bias_window').value))
        self._gyro_z_bias = 0.0
        self._bias_samples = 0
        self._still_since = None   # stamp (s) the wheels first read stationary
        self._is_still = False

        twist_cov = [float(v) for v in p('lidar_twist_cov').value]
        if len(twist_cov) != 3 or not all(math.isfinite(v) and v > 0.0 for v in twist_cov):
            raise ValueError('lidar_twist_cov must be three positive [vx, vy, vyaw] variances')
        self._tw_cov_vx, self._tw_cov_vy, self._tw_cov_vyaw = twist_cov
        self._tw_max_dt = float(p('lidar_twist_max_dt').value)
        self._vx_gate = float(p('twist_vx_gate').value)
        self._wz_gate = float(p('twist_wz_gate').value)
        self._wheel_vx_floor = float(p('wheel_vx_cov_floor').value)
        self._stall_enabled = bool(p('stall_detection').value)
        self._stall_lidar_speed = float(p('stall_lidar_speed').value)
        self._stall_wheel_speed = float(p('stall_wheel_speed').value)
        self._stall_min_steps = max(1, int(p('stall_min_steps').value))
        self._stall_steps = 0          # consecutive FAST-LIO2 steps meeting the stall test
        self._wheels_stalled = False   # while True, wheel vx is rejected
        self._n_stall_kept = 0
        self._prev_lidar = None                      # (stamp, x, y, yaw)
        self._wheel_hist = collections.deque()       # (stamp, vx)
        self._gyro_hist = collections.deque()        # (stamp, bias-corrected wz)
        self._n_twist = self._n_vx_gated = self._n_wz_gated = 0

        imu_in = p('imu_in').value
        imu_out = p('imu_out').value
        lidar_in = p('lidar_odom_in').value
        lidar_out = p('lidar_odom_out').value

        self._imu_pub = self.create_publisher(Imu, imu_out, 10)
        self._lidar_pub = self.create_publisher(Odometry, lidar_out, 10)
        self._twist_pub = self.create_publisher(Odometry, p('lidar_twist_out').value, 10)
        self._wheel_pub = self.create_publisher(Odometry, p('wheel_odom_out').value, 20)
        self._imu_sub = self.create_subscription(
            Imu, imu_in, self._on_imu, qos_profile_sensor_data)
        self._lidar_sub = self.create_subscription(
            Odometry, lidar_in, self._on_lidar, 20)
        self._wheel_sub = self.create_subscription(
            Odometry, p('wheel_odom_in').value, self._on_wheel, 50)

        self.get_logger().info(
            f'IMU {imu_in} -> {imu_out} (gyro cov x{self._imu_scale:g}, '
            f'floor {self._imu_floor:g}, bias removal '
            f'{"on" if self._bias_enabled else "off"}); lidar {lidar_in} -> '
            f'{lidar_out} + gated body twist -> {p("lidar_twist_out").value} '
            f'(cov [{self._tw_cov_vx:g}, {self._tw_cov_vy:g}, {self._tw_cov_vyaw:g}], '
            f'gates vx {self._vx_gate:g} m/s, wz {self._wz_gate:g} rad/s); '
            f'wheel -> {p("wheel_odom_out").value} (vx var floor {self._wheel_vx_floor:g})')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _on_wheel(self, msg: Odometry):
        tw = msg.twist.twist
        if not _all_finite((tw.linear.x, tw.angular.z, *msg.twist.covariance)):
            return
        stamp = _stamp(msg)
        self._wheel_hist.append((stamp, tw.linear.x))
        while self._wheel_hist and self._wheel_hist[0][0] < stamp - _HISTORY_S:
            self._wheel_hist.popleft()

        cov = list(msg.twist.covariance)
        cov[_TW_VX] = max(cov[_TW_VX], self._wheel_vx_floor)
        if self._wheels_stalled:
            cov[_TW_VX] = _REJECTED_VAR
        msg.twist.covariance = cov
        self._wheel_pub.publish(msg)

        still = (abs(tw.linear.x) < self._still_thresh and
                 abs(tw.angular.z) < self._still_thresh)
        if not still:
            self._still_since = None
            self._is_still = False
            return
        if self._still_since is None:
            self._still_since = stamp
        self._is_still = (stamp - self._still_since) >= self._still_min_dur

    def _on_imu(self, msg: Imu):
        av = msg.angular_velocity
        cov = list(msg.angular_velocity_covariance)
        if not _all_finite((av.x, av.y, av.z, *cov)):
            self.get_logger().warn(
                'Dropping non-finite IMU sample (NaN/Inf in angular_velocity '
                'or its covariance) -- not forwarding to the EKF.',
                throttle_duration_sec=5.0)
            return
        if self._bias_enabled:
            if self._is_still:
                # Running mean up to the window size, then an exponential
                # average with the same effective length.
                self._bias_samples = min(self._bias_samples + 1, self._bias_window)
                self._gyro_z_bias += (av.z - self._gyro_z_bias) / self._bias_samples
            msg.angular_velocity.z = av.z - self._gyro_z_bias
            self.get_logger().info(
                f'gyro z bias estimate {self._gyro_z_bias:+.5f} rad/s '
                f'({self._bias_samples} stationary samples)',
                throttle_duration_sec=30.0)
        stamp = _stamp(msg)
        self._gyro_hist.append((stamp, msg.angular_velocity.z))
        while self._gyro_hist and self._gyro_hist[0][0] < stamp - _HISTORY_S:
            self._gyro_hist.popleft()
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
        self._publish_gated_twist(msg)

    def _wheel_vx_over(self, t0, t1):
        """Mean wheel vx over [t0, t1], or None if not covered yet."""
        vals = [v for t, v in self._wheel_hist if t0 <= t <= t1]
        if not vals:
            if not self._wheel_hist or self._wheel_hist[-1][0] < t1:
                return None
            # Interval shorter than the wheel period: nearest sample.
            return min(self._wheel_hist, key=lambda s: abs(s[0] - 0.5 * (t0 + t1)))[1]
        return sum(vals) / len(vals)

    def _gyro_dyaw_over(self, t0, t1):
        """Integrated bias-corrected gyro yaw over [t0, t1], or None."""
        if not self._gyro_hist or self._gyro_hist[-1][0] < t1 or self._gyro_hist[0][0] > t0:
            return None
        dyaw, prev_t = 0.0, None
        for t, wz in self._gyro_hist:
            if t <= t0:
                prev_t = t
                continue
            if prev_t is None:
                prev_t = t0
            seg_end = min(t, t1)
            dyaw += wz * (seg_end - max(prev_t, t0))
            prev_t = t
            if t >= t1:
                break
        return dyaw

    def _publish_gated_twist(self, msg: Odometry):
        stamp = _stamp(msg)
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        prev, self._prev_lidar = self._prev_lidar, (stamp, x, y, yaw)
        if prev is None:
            return
        dt = stamp - prev[0]
        if dt <= 0.0 or dt > self._tw_max_dt:
            return
        dx, dy = x - prev[1], y - prev[2]
        dyaw = math.atan2(math.sin(yaw - prev[3]), math.cos(yaw - prev[3]))
        h = prev[3] + 0.5 * dyaw          # midpoint heading
        c, s = math.cos(h), math.sin(h)
        vx = (c * dx + s * dy) / dt
        vy = (-s * dx + c * dy) / dt
        vyaw = dyaw / dt
        if not _all_finite((vx, vy, vyaw)):
            return

        var_vx, var_vy, var_vyaw = self._tw_cov_vx, self._tw_cov_vy, self._tw_cov_vyaw
        wheel_vx = self._wheel_vx_over(prev[0], stamp)
        lidar_still = (self._stall_enabled and wheel_vx is not None and
                       abs(vx) < self._stall_lidar_speed and
                       abs(wheel_vx) > self._stall_wheel_speed)
        self._stall_steps = self._stall_steps + 1 if lidar_still else 0
        self._wheels_stalled = self._stall_steps >= self._stall_min_steps
        if lidar_still:
            # Wheels report motion FAST-LIO2 does not see: stall / wheel spin,
            # not a corridor slide. Keep FAST-LIO2 (see STALL in the header).
            self._n_stall_kept += 1
        elif wheel_vx is not None and abs(vx - wheel_vx) > self._vx_gate:
            # Along-corridor slide (or similar): drop FAST-LIO2's translation
            # for this step; the EKF follows wheel vx (and vy = 0).
            var_vx = var_vy = _REJECTED_VAR
            self._n_vx_gated += 1
        gyro_dyaw = self._gyro_dyaw_over(prev[0], stamp)
        if gyro_dyaw is not None and abs(vyaw - gyro_dyaw / dt) > self._wz_gate:
            var_vyaw = _REJECTED_VAR
            self._n_wz_gated += 1
        self._n_twist += 1
        self.get_logger().info(
            f'lidar twist: {self._n_twist} steps, vx gated {self._n_vx_gated}, '
            f'vyaw gated {self._n_wz_gated}, kept over wheels (stall) {self._n_stall_kept}',
            throttle_duration_sec=30.0)

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.orientation.w = 1.0
        out.twist.twist.linear.x = vx
        out.twist.twist.linear.y = vy
        out.twist.twist.angular.z = vyaw
        tcov = [0.0] * 36
        tcov[_TW_VX], tcov[_TW_VY], tcov[_TW_VYAW] = var_vx, var_vy, var_vyaw
        for i in (14, 21, 28):   # vz, vroll, vpitch: not fused, keep valid
            tcov[i] = _REJECTED_VAR
        out.twist.covariance = tcov
        self._twist_pub.publish(out)


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
