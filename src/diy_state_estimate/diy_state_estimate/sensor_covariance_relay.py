#!/usr/bin/env python3
"""
sensor_covariance_relay — re-stamps sensor covariances before EKF fusion.

WHY THIS EXISTS
───────────────
robot_localization has no per-sensor "weight" parameter: it trusts whatever
covariance each message carries. Several of our inputs report covariances
that would let them completely dominate the fused estimate, or need
independent sanity-checking before they can be trusted at all:

  * /zed/zed_node/imu/data — the ZED's own gyro. FAST-LIO2 and the ZED's
                    visual-inertial odometry both already fuse this same
                    physical IMU internally, so it is NOT an independent
                    sensor relative to either twist source below. It is still
                    the best available high-rate rotation-rate measurement
                    (200 Hz, no pose-differentiation noise), so it remains
                    the EKF's primary vyaw source; what it can no longer do
                    is serve as independent cross-validation. Gating a
                    twist's own computed vyaw against it still catches real
                    per-source errors (scan-matching yaw ambiguity, visual
                    feature mismatch) that have nothing to do with the gyro
                    itself, but it is not a second vote in a consensus sense.
  * /Odometry, /zed/zed_node/odom — FAST-LIO2 and the ZED VIO node each
                    export their own internal filter's covariance, which is
                    very small once the map/scene is well tracked. Neither
                    filter knows whether the *scene* is static: a moving
                    obstacle in front of a stationary robot can show up as
                    confident ego-motion in both. The wheel encoders (which
                    say v = 0) need real weight to pull against that.

This node subscribes to all of these, applies a floor (and optional scale) to
the relevant covariance blocks, and republishes on *_ekf topics that the EKF
consumes instead of the raw ones. Nothing else about the messages is touched,
so FAST-LIO2, the ZED node, map_localizer, etc. keep reading the raw topics
unchanged.

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
robot_localization does not estimate gyro bias, and the odom EKF takes its
heading only from the IMU yaw rate (config/ekf_fusion.yaml). Whenever
/wheel_odom reports the robot standing still (|vx| and |vyaw| below
stationary_speed_threshold for at least stationary_min_duration), this node
averages angular_velocity.z into a bias estimate and subtracts it from every
forwarded IMU sample, the same way it previously removed the ACEINNA gyro's
z bias (~-0.0017 rad/s, ~23 deg of heading drift over a 230 s run if left
in) -- re-check this figure once bags exist with the ZED gyro as the source.

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

GATED BODY TWIST FROM FAST-LIO2 AND ZED VIO (lidar_twist_out / vio_twist_out)
──────────────────────────────────────────────────────────────────────────────
Neither FAST-LIO2's /Odometry nor the ZED node's /zed/zed_node/odom carries a
trustworthy twist directly, so this node differentiates consecutive poses
from EACH into its own body-frame (vx, vy, vyaw) and publishes them as two
SEPARATE, independently-gated EKF inputs -- lidar_twist_out and
vio_twist_out. There is no cross-check between the two (LIO is not compared
against VIO): each is only gated against the wheels/gyro, exactly like the
single-source version of this used to work. Per component:

  * vx vs wheel forward speed: in long straight parallel-walled sections the
    LiDAR cannot observe along-corridor motion and FAST-LIO2 was confirmed to
    slide backward at up to ~1.5 m/s while the wheels read ~0.4 m/s forward.
    The ZED VIO node has an analogous failure mode in low-texture/low-light
    scenes, so its vx gets the same treatment.
  * vyaw vs bias-corrected gyro yaw rate (see the caveat above: this is a
    per-source diagnostic, not independent confirmation, since both
    LIO and VIO already consume this same gyro internally).

A component that disagrees by more than its vx_gate / wz_gate is published
with a huge variance, so the EKF effectively ignores it for that step and
follows wheels / gyro instead. The check must be done here rather than with
the EKF's own Mahalanobis gate: that gate compares against the fused state,
which the twist input dominates, so a slide that builds up gradually drags
the state with it and never trips. Offline on run-2 this gating removed the
FAST-LIO2 corridor ghost and cut the end-of-lap error from 11.2 m to 3.0 m.

  lidar_twist_cov     FAST-LIO2 [vx, vy, vyaw] variances for accepted
                      components. Default [0.0004, 0.0004, 0.05]: vyaw
                      deliberately weak, the gyro is the main turn-rate
                      source.
  twist_vx_gate       FAST-LIO2 vx/vy gate, m/s. Default 0.2.
  twist_wz_gate       FAST-LIO2 vyaw gate, rad/s. Default 0.3.
  lidar_twist_max_dt  skip differentiation across FAST-LIO2 pose gaps longer
                      than this.
  vio_twist_cov       ZED VIO [vx, vy, vyaw] variances for accepted
                      components. Default [0.0004, 0.0004, 0.05] (same
                      starting point as FAST-LIO2; retune once VIO-specific
                      bags exist).
  vio_vx_gate         ZED VIO vx/vy gate, m/s. Default 0.2.
  vio_wz_gate         ZED VIO vyaw gate, rad/s. Default 0.3.
  vio_twist_max_dt    skip differentiation across ZED VIO pose gaps longer
                      than this.

WHEEL STALL / SPIN (one-sided vx check)
───────────────────────────────────────
The vx check above assumes the twist source is the one that is wrong
(corridor slide: it reports motion the wheels do not). The opposite also
happens: on the 2026-09-27 obstacle-course run-2 the robot got stuck while
manoeuvring and the wheels reported ~0.5 m of reverse motion three times in
a row while an independent LiDAR scan match showed it moved 1-3 cm. The EKF
followed the wheels and put the robot ~1.5 m off, which later drew the
northbound tunnel leg on top of the southbound wide-section leg.

So when a twist source says the robot is still (|vx| < stall_lidar_speed /
stall_vio_speed) while the wheels say it is moving (|vx| > stall_wheel_speed),
that source is kept rather than gated. If that persists for stall_min_steps
steps of that source (skips the single-step lag when the robot starts
moving), the wheels' vx is rejected until it ends. Either source alone can
trigger the wheel rejection.

  stall_detection    enable/disable, applies to both sources. Default true.
  stall_lidar_speed  FAST-LIO2 stillness threshold, m/s. Default 0.05.
  stall_vio_speed    ZED VIO stillness threshold, m/s. Default 0.05.
  stall_wheel_speed  m/s. Default 0.15.
  stall_min_steps    Default 3 (0.3 s at 10 Hz for FAST-LIO2; faster at
                     the ZED VIO node's own rate).

WHEEL ODOMETRY (wheel_odom_out)
───────────────────────────────
/wheel_odom republished with its vx variance floored to wheel_vx_cov_floor.
The driver reports 0.001 at 100 Hz, which would out-vote either 10-Hz-class
twist source above; the floor (default 0.04) makes the wheels a fallback for
forward speed rather than the main source.

ACEINNA IMU (imu1_out) — OPTIONAL, GYRO (vyaw) ONLY
────────────────────────────────────────────────────
The ACEINNA IMU is physically mounted at imu_link (robot_description/urdf/
imu.urdf.xacro), close to the lidar/camera stack but offset from base_link's
rotation center. That mounting position matters differently per state:

  * Angular velocity (gyro, incl. vyaw) is the SAME everywhere on a rigid
    body regardless of mounting offset -- so it is safe, and worthwhile, to
    fuse directly. It is also the one genuinely independent physical IMU in
    this stack: FAST-LIO2 and the ZED VIO node both already internally
    consume the SAME ZED gyro (imu0), so cross-checking either twist's vyaw
    against imu0 is not true independent confirmation. ACEINNA is.
  * Raw linear acceleration (ax/ay/az) is NOT safe to fuse from an
    off-center IMU without lever-arm (centripetal/tangential) compensation:
    during any real angular velocity/acceleration, an off-axis accelerometer
    reads extra apparent acceleration that does not represent the vehicle's
    own translation. robot_localization does not apply that compensation
    automatically. Hence this relay never forwards ACEINNA (or ZED) linear
    acceleration to the EKF.
  * Absolute roll/pitch are irrelevant here anyway (two_d_mode: true).

Disabled by default (aceinna_enable: false) since no ACEINNA driver is
currently launched in this repo (see ekf_fusion.launch.py header) -- the
EKF's imu1 input simply stays idle (no error) until this is turned on and
a real driver publishes on aceinna_imu_in.

  aceinna_enable           enable/disable the ACEINNA relay + EKF input.
  aceinna_imu_in           raw ACEINNA topic. Default '/imu/data' (this
                          repo's historical ACEINNA driver topic; FAST-LIO2's
                          common.imu_topic still defaults to this).
  aceinna_imu_out          re-stamped topic the EKF's imu1 consumes.
  aceinna_gyro_cov_scale   multiplier applied BEFORE the floor. Default 1.0.
  aceinna_gyro_cov_floor   minimum variance on each gyro axis, (rad/s)^2.
                          Same starting point as imu_gyro_cov_floor; retune
                          once real ACEINNA noise figures are known.
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


class _TwistGate:
    """Differentiates consecutive poses from ONE odometry source into a
    gated body twist (vx, vy, vyaw), publishing huge variance on whichever
    component disagrees with the wheels/gyro that step.

    One instance per source (FAST-LIO2, ZED VIO). Instances never compare
    against each other -- each is only gated against wheel vx and the gyro
    yaw rate, both supplied by the relay via callables so this class stays
    independent of how those histories are stored. See the module docstring
    ("GATED BODY TWIST...") for the rationale and every parameter.
    """

    def __init__(self, name, publisher, cov, max_dt, vx_gate, wz_gate,
                 stall_enabled, stall_own_speed, stall_wheel_speed, stall_min_steps):
        self._name = name
        self._pub = publisher
        self._cov_vx, self._cov_vy, self._cov_vyaw = cov
        self._max_dt = max_dt
        self._vx_gate = vx_gate
        self._wz_gate = wz_gate
        self._stall_enabled = stall_enabled
        self._stall_own_speed = stall_own_speed
        self._stall_wheel_speed = stall_wheel_speed
        self._stall_min_steps = max(1, int(stall_min_steps))

        self._prev = None            # (stamp, x, y, yaw)
        self._stall_steps = 0        # consecutive steps meeting the stall test
        self.stalled = False         # while True, the caller should reject wheel vx
        self._n_twist = self._n_vx_gated = self._n_wz_gated = self._n_stall_kept = 0

    def process(self, msg, wheel_vx_over, gyro_dyaw_over, logger):
        """msg: Odometry with a pose to differentiate.
        wheel_vx_over, gyro_dyaw_over: callables (t0, t1) -> mean wheel vx /
        integrated gyro yaw over that interval, or None if not covered.
        Returns the gated-twist Odometry to publish, or None (first sample /
        pose gap too long / non-finite result -- nothing published this step).
        """
        stamp = _stamp(msg)
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        prev, self._prev = self._prev, (stamp, x, y, yaw)
        if prev is None:
            return None
        dt = stamp - prev[0]
        if dt <= 0.0 or dt > self._max_dt:
            return None
        dx, dy = x - prev[1], y - prev[2]
        dyaw = math.atan2(math.sin(yaw - prev[3]), math.cos(yaw - prev[3]))
        h = prev[3] + 0.5 * dyaw          # midpoint heading
        c, s = math.cos(h), math.sin(h)
        vx = (c * dx + s * dy) / dt
        vy = (-s * dx + c * dy) / dt
        vyaw = dyaw / dt
        if not _all_finite((vx, vy, vyaw)):
            return None

        var_vx, var_vy, var_vyaw = self._cov_vx, self._cov_vy, self._cov_vyaw
        wheel_vx = wheel_vx_over(prev[0], stamp)
        source_still = (self._stall_enabled and wheel_vx is not None and
                         abs(vx) < self._stall_own_speed and
                         abs(wheel_vx) > self._stall_wheel_speed)
        self._stall_steps = self._stall_steps + 1 if source_still else 0
        self.stalled = self._stall_steps >= self._stall_min_steps
        if source_still:
            # Wheels report motion this source does not see: stall / wheel
            # spin, not a corridor slide. Keep this source (see STALL in the
            # module docstring).
            self._n_stall_kept += 1
        elif wheel_vx is not None and abs(vx - wheel_vx) > self._vx_gate:
            # Along-corridor slide (or similar): drop this source's
            # translation for this step; the EKF follows wheel vx (vy = 0).
            var_vx = var_vy = _REJECTED_VAR
            self._n_vx_gated += 1
        gyro_dyaw = gyro_dyaw_over(prev[0], stamp)
        if gyro_dyaw is not None and abs(vyaw - gyro_dyaw / dt) > self._wz_gate:
            var_vyaw = _REJECTED_VAR
            self._n_wz_gated += 1
        self._n_twist += 1
        logger.info(
            f'{self._name} twist: {self._n_twist} steps, vx gated {self._n_vx_gated}, '
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
        return out

    def publish(self, out):
        self._pub.publish(out)


class SensorCovarianceRelay(Node):

    def __init__(self):
        super().__init__('sensor_covariance_relay')

        self.declare_parameter('imu_in', '/zed/zed_node/imu/data')
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
        self.declare_parameter('stall_lidar_speed', 0.05)

        self.declare_parameter('vio_odom_in', '/zed/zed_node/odom')
        self.declare_parameter('vio_twist_out', '/vio_twist_ekf')
        self.declare_parameter('vio_twist_cov', [0.0004, 0.0004, 0.05])
        self.declare_parameter('vio_twist_max_dt', 0.5)
        self.declare_parameter('vio_vx_gate', 0.2)
        self.declare_parameter('vio_wz_gate', 0.3)
        self.declare_parameter('stall_vio_speed', 0.05)

        self.declare_parameter('stall_detection', True)
        self.declare_parameter('stall_wheel_speed', 0.15)
        self.declare_parameter('stall_min_steps', 3)
        self.declare_parameter('wheel_odom_out', '/wheel_odom_ekf')
        self.declare_parameter('wheel_vx_cov_floor', 0.04)

        # Optional second, genuinely-independent physical IMU (see the
        # ACEINNA module docstring section above). Disabled by default.
        self.declare_parameter('aceinna_enable', False)
        self.declare_parameter('aceinna_imu_in', '/imu/data')
        self.declare_parameter('aceinna_imu_out', '/imu/data_aceinna_ekf')
        self.declare_parameter('aceinna_gyro_cov_scale', 1.0)
        self.declare_parameter('aceinna_gyro_cov_floor', 0.004)

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

        self._aceinna_enabled = bool(p('aceinna_enable').value)
        self._aceinna_scale = float(p('aceinna_gyro_cov_scale').value)
        self._aceinna_floor = float(p('aceinna_gyro_cov_floor').value)
        self._aceinna_gyro_z_bias = 0.0
        self._aceinna_bias_samples = 0

        self._wheel_vx_floor = float(p('wheel_vx_cov_floor').value)
        stall_enabled = bool(p('stall_detection').value)
        stall_wheel_speed = float(p('stall_wheel_speed').value)
        stall_min_steps = int(p('stall_min_steps').value)
        self._wheels_stalled = False   # while True, wheel vx is rejected
        self._wheel_hist = collections.deque()       # (stamp, vx)
        self._gyro_hist = collections.deque()        # (stamp, bias-corrected wz)

        def _twist_cov(name, values):
            cov = [float(v) for v in values]
            if len(cov) != 3 or not all(math.isfinite(v) and v > 0.0 for v in cov):
                raise ValueError(f'{name} must be three positive [vx, vy, vyaw] variances')
            return cov

        imu_in = p('imu_in').value
        imu_out = p('imu_out').value
        lidar_in = p('lidar_odom_in').value
        lidar_out = p('lidar_odom_out').value
        vio_in = p('vio_odom_in').value

        self._imu_pub = self.create_publisher(Imu, imu_out, 10)
        self._lidar_pub = self.create_publisher(Odometry, lidar_out, 10)
        self._wheel_pub = self.create_publisher(Odometry, p('wheel_odom_out').value, 20)
        self._aceinna_pub = (
            self.create_publisher(Imu, p('aceinna_imu_out').value, 10)
            if self._aceinna_enabled else None)

        # One independent _TwistGate per body-twist source. Neither is
        # compared against the other -- each is only gated against wheel vx
        # and the gyro yaw rate (see the module docstring).
        self._lidar_gate = _TwistGate(
            'FAST-LIO2',
            self.create_publisher(Odometry, p('lidar_twist_out').value, 10),
            _twist_cov('lidar_twist_cov', p('lidar_twist_cov').value),
            float(p('lidar_twist_max_dt').value),
            float(p('twist_vx_gate').value),
            float(p('twist_wz_gate').value),
            stall_enabled, float(p('stall_lidar_speed').value),
            stall_wheel_speed, stall_min_steps)
        self._vio_gate = _TwistGate(
            'ZED VIO',
            self.create_publisher(Odometry, p('vio_twist_out').value, 10),
            _twist_cov('vio_twist_cov', p('vio_twist_cov').value),
            float(p('vio_twist_max_dt').value),
            float(p('vio_vx_gate').value),
            float(p('vio_wz_gate').value),
            stall_enabled, float(p('stall_vio_speed').value),
            stall_wheel_speed, stall_min_steps)

        self._imu_sub = self.create_subscription(
            Imu, imu_in, self._on_imu, qos_profile_sensor_data)
        self._lidar_sub = self.create_subscription(
            Odometry, lidar_in, self._on_lidar, 20)
        self._vio_sub = self.create_subscription(
            Odometry, vio_in, self._on_vio, 20)
        self._wheel_sub = self.create_subscription(
            Odometry, p('wheel_odom_in').value, self._on_wheel, 50)
        self._aceinna_sub = (
            self.create_subscription(
                Imu, p('aceinna_imu_in').value, self._on_aceinna_imu,
                qos_profile_sensor_data)
            if self._aceinna_enabled else None)

        self.get_logger().info(
            f'IMU {imu_in} -> {imu_out} (gyro cov x{self._imu_scale:g}, '
            f'floor {self._imu_floor:g}, bias removal '
            f'{"on" if self._bias_enabled else "off"}); '
            f'lidar {lidar_in} -> {lidar_out} + gated body twist -> '
            f'{p("lidar_twist_out").value}; '
            f'vio {vio_in} -> gated body twist -> {p("vio_twist_out").value} '
            f'(no cross-check between the two); '
            f'wheel -> {p("wheel_odom_out").value} (vx var floor {self._wheel_vx_floor:g}); '
            f'aceinna {"enabled -> " + p("aceinna_imu_out").value if self._aceinna_enabled else "disabled"}')

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

    def _on_aceinna_imu(self, msg: Imu):
        """Bias-corrected, floored ACEINNA gyro -> imu1 (vyaw only downstream;
        see the ACEINNA module docstring section for why only the gyro is
        forwarded, never linear acceleration)."""
        av = msg.angular_velocity
        cov = list(msg.angular_velocity_covariance)
        if not _all_finite((av.x, av.y, av.z, *cov)):
            self.get_logger().warn(
                'Dropping non-finite ACEINNA IMU sample (NaN/Inf in '
                'angular_velocity or its covariance) -- not forwarding to the EKF.',
                throttle_duration_sec=5.0)
            return
        if self._bias_enabled and self._is_still:
            self._aceinna_bias_samples = min(self._aceinna_bias_samples + 1, self._bias_window)
            self._aceinna_gyro_z_bias += (
                (av.z - self._aceinna_gyro_z_bias) / self._aceinna_bias_samples)
        msg.angular_velocity.z = av.z - self._aceinna_gyro_z_bias
        if cov[0] < 0.0:
            cov = [0.0] * 9
        cov = [c * self._aceinna_scale for c in cov]
        for i in (0, 4, 8):
            cov[i] = max(cov[i], self._aceinna_floor)
        msg.angular_velocity_covariance = cov
        self._aceinna_pub.publish(msg)

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
        self._process_gate(self._lidar_gate, msg)

    def _on_vio(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        if not _all_finite((pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)):
            self.get_logger().warn(
                'Dropping non-finite ZED VIO odometry sample (NaN/Inf in '
                'pose) -- not forwarding to the EKF.',
                throttle_duration_sec=5.0)
            return
        self._process_gate(self._vio_gate, msg)

    def _process_gate(self, gate: '_TwistGate', msg: Odometry):
        """Run one _TwistGate over msg, publish its output (if any), and OR
        its stall flag into the shared wheel-vx rejection."""
        out = gate.process(msg, self._wheel_vx_over, self._gyro_dyaw_over,
                            self.get_logger())
        self._wheels_stalled = self._lidar_gate.stalled or self._vio_gate.stalled
        if out is not None:
            gate.publish(out)

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
