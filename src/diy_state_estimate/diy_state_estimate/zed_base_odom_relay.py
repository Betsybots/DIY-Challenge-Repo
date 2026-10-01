#!/usr/bin/env python3
"""
zed_base_odom_relay — ZED2i visual odometry re-expressed for the robot base
═══════════════════════════════════════════════════════════════════════════
The ZED wrapper tracks its own `camera_link` (zed_camera_link): /zed/zed_node/odom
is the pose of the CAMERA in the ZED's odom frame, which starts at the camera's
own start pose. robot_localization and map_localizer both treat an Odometry pose
as the pose of the message's base frame, ignoring child_frame_id for the pose
part, so fusing the raw message moves base_footprint along the camera's arc
(error 2·d·sin(Δyaw/2) for a camera d metres ahead of the base: ~0.30 m after a
90° turn for this robot's 0.21 m offset).

This node republishes it as the pose/twist of `base_frame`:
    C = base_frame -> camera_frame  (static, from TF / URDF, or parameters)
    P = camera pose relative to the camera's first reading
    B = C · P · C⁻¹                 (base pose relative to the base's start)
    ω_base = R_C ω_cam,  v_base = R_C v_cam − ω_base × t_C
So the output starts at identity like any wheel/EKF odometry.

Extras:
  * two_d_mode (default true): z, roll, pitch forced to 0 — the ZED's height
    drifts ~0.2 m per 5 m on this robot.
  * Jump re-anchoring: the ZED wrapper's area_memory / loop closure can snap its
    odometry (a 3.45 m jump back to the origin was seen in the 2026-09-28 bag).
    A step larger than max_speed·dt + jump_margin is treated as such a reset and
    the output is re-anchored so it stays continuous.
  * Covariances: the ZED publishes zero twist covariance (robot_localization
    would treat that as near-certain); twist_covariance sets real values.
  * publish_tf (default false): broadcast odom -> base_frame from this output.
    Only enable it when nothing else (EKF) owns that transform.

Recommended ZED wrapper settings when using this node:
  pos_tracking.publish_tf: false       (it would publish odom -> zed_camera_link)
  pos_tracking.publish_map_tf: false   (map -> odom belongs to map_localizer)
  pos_tracking.reset_odom_with_loop_closure: false   (or area_memory: false)
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def quat_to_rot(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_to_quat(r):
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([r[0, 0], r[1, 1], r[2, 2]]))
    if i == 0:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        return (0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        return ((r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s)
    s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
    return ((r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s, (r[1, 0] - r[0, 1]) / s)


def rpy_to_rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def make_tf(r, t):
    m = np.eye(4)
    m[:3, :3] = r
    m[:3, 3] = t
    return m


def inv_tf(m):
    r = m[:3, :3].T
    return make_tf(r, -r @ m[:3, 3])


def flatten(m):
    """Keep x, y, yaw; zero z, roll, pitch."""
    yaw = math.atan2(m[1, 0], m[0, 0])
    return make_tf(rpy_to_rot(0.0, 0.0, yaw), [m[0, 3], m[1, 3], 0.0])


def camera_to_base(p_rel, c, c_inv):
    """Base pose relative to its start, given the camera pose relative to its start."""
    return c @ p_rel @ c_inv


def camera_twist_to_base(v_cam, w_cam, c):
    """Rigid-body twist transfer from the camera frame to the base frame."""
    r, t = c[:3, :3], c[:3, 3]
    w_base = r @ w_cam
    v_base = r @ v_cam - np.cross(w_base, t)
    return v_base, w_base


class ZedBaseOdomRelay(Node):
    def __init__(self):
        super().__init__('zed_base_odom_relay')
        p = self.declare_parameter
        self._in_topic = p('zed_odom_topic', '/zed/zed_node/odom').value
        self._out_topic = p('output_topic', '/zed/odom_base').value
        self._odom_frame = p('odom_frame', 'odom').value
        self._base_frame = p('base_frame', 'base_footprint').value
        self._camera_frame = p('camera_frame', '').value  # '' = use msg.child_frame_id
        # Fallback base_frame -> camera_frame if TF has no such transform:
        # base_link -> zed_camera_link (8.25, 0, 6.25) in = (0.2096, 0, 0.1588) m;
        # base_footprint -> base_link adds the wheel radius (0.0667 m) in z.
        self._fallback_xyz = list(p('camera_offset_xyz', [0.2096, 0.0, 0.2255]).value)
        self._fallback_rpy = list(p('camera_offset_rpy', [0.0, 0.0, 0.0]).value)
        self._tf_wait_s = float(p('tf_wait_s', 3.0).value)
        self._two_d = bool(p('two_d_mode', True).value)
        self._max_speed = float(p('max_speed', 3.0).value)
        self._jump_margin = float(p('jump_margin', 0.2).value)
        self._max_yaw_rate = float(p('max_yaw_rate', 4.0).value)
        self._pose_cov_scale = float(p('pose_covariance_scale', 1.0).value)
        self._twist_cov = list(p('twist_covariance', [0.04, 0.04, 0.04, 0.01, 0.01, 0.01]).value)
        self._publish_tf = bool(p('publish_tf', False).value)
        if len(self._fallback_xyz) != 3 or len(self._fallback_rpy) != 3 or len(self._twist_cov) != 6:
            raise ValueError('camera_offset_xyz/rpy need 3 values, twist_covariance needs 6')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_br = TransformBroadcaster(self) if self._publish_tf else None
        self._pub = self.create_publisher(Odometry, self._out_topic, 20)
        self.create_subscription(Odometry, self._in_topic, self._on_odom, 50)

        self._c = None            # base -> camera
        self._c_inv = None
        self._start_time = None
        self._p0_inv = None       # inverse of the camera's first pose
        self._anchor = np.eye(4)  # re-anchoring after ZED resets
        self._last_raw = None     # last B before anchoring (raw chain)
        self._last_out = None
        self._last_stamp = None
        self._n_resets = 0
        self.get_logger().info(
            f'{self._in_topic} (camera) -> {self._out_topic} ({self._odom_frame} -> {self._base_frame}), '
            f'two_d_mode={self._two_d}, publish_tf={self._publish_tf}')

    def _resolve_camera_transform(self, cam_frame, now):
        try:
            tf = self._tf_buffer.lookup_transform(self._base_frame, cam_frame, Time())
            tr, q = tf.transform.translation, tf.transform.rotation
            c = make_tf(quat_to_rot(q.x, q.y, q.z, q.w), [tr.x, tr.y, tr.z])
            source = f'TF {self._base_frame} -> {cam_frame}'
        except Exception:  # noqa: BLE001 — any TF failure falls back to parameters
            if self._start_time is None:
                self._start_time = now
            if (now - self._start_time) < Duration(seconds=self._tf_wait_s):
                return False
            c = make_tf(rpy_to_rot(*self._fallback_rpy), self._fallback_xyz)
            source = f'parameters (no TF {self._base_frame} -> {cam_frame} after {self._tf_wait_s:.0f} s)'
        self._c, self._c_inv = c, inv_tf(c)
        t = c[:3, 3]
        self.get_logger().info(f'camera offset from {source}: xyz=({t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}) m')
        return True

    def _on_odom(self, msg: Odometry):
        stamp = Time.from_msg(msg.header.stamp)
        if self._c is None and not self._resolve_camera_transform(
                self._camera_frame or msg.child_frame_id, self.get_clock().now()):
            return

        q, pos = msg.pose.pose.orientation, msg.pose.pose.position
        p_abs = make_tf(quat_to_rot(q.x, q.y, q.z, q.w), [pos.x, pos.y, pos.z])
        if self._p0_inv is None:
            self._p0_inv = inv_tf(p_abs)
        raw = camera_to_base(self._p0_inv @ p_abs, self._c, self._c_inv)
        if self._two_d:
            raw = flatten(raw)

        if self._last_raw is not None and self._last_stamp is not None:
            dt = max((stamp - self._last_stamp).nanoseconds * 1e-9, 1e-3)
            step = inv_tf(self._last_raw) @ raw
            step_dist = float(np.linalg.norm(step[:3, 3]))
            step_yaw = abs(math.atan2(step[1, 0], step[0, 0]))
            if (step_dist > self._max_speed * dt + self._jump_margin or
                    step_yaw > self._max_yaw_rate * dt + math.radians(10.0)):
                # ZED reset / loop-closure snap: continue from the last output.
                self._anchor = self._last_out @ inv_tf(raw)
                self._n_resets += 1
                self.get_logger().warn(
                    f'ZED odometry jumped {step_dist:.2f} m / {math.degrees(step_yaw):.1f} deg in '
                    f'{dt:.3f} s — treated as a ZED reset, output re-anchored (#{self._n_resets})')
        out = self._anchor @ raw
        self._last_raw, self._last_out, self._last_stamp = raw, out, stamp

        tw = msg.twist.twist
        v_base, w_base = camera_twist_to_base(
            np.array([tw.linear.x, tw.linear.y, tw.linear.z]),
            np.array([tw.angular.x, tw.angular.y, tw.angular.z]), self._c)
        if self._two_d:
            v_base[2] = 0.0
            w_base[0] = w_base[1] = 0.0

        o = Odometry()
        o.header.stamp = msg.header.stamp
        o.header.frame_id = self._odom_frame
        o.child_frame_id = self._base_frame
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = out[:3, 3]
        (o.pose.pose.orientation.x, o.pose.pose.orientation.y,
         o.pose.pose.orientation.z, o.pose.pose.orientation.w) = rot_to_quat(out[:3, :3])
        cov = [0.0] * 36
        for i in range(6):
            cov[i * 7] = max(msg.pose.covariance[i * 7], 1e-6) * self._pose_cov_scale
        if self._two_d:
            cov[14] = cov[21] = cov[28] = 1e-6
        o.pose.covariance = cov
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = v_base
        o.twist.twist.angular.x, o.twist.twist.angular.y, o.twist.twist.angular.z = w_base
        tcov = [0.0] * 36
        for i, v in enumerate(self._twist_cov):
            tcov[i * 7] = v
        o.twist.covariance = tcov
        self._pub.publish(o)

        if self._tf_br is not None:
            t = TransformStamped()
            t.header = o.header
            t.child_frame_id = self._base_frame
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = out[:3, 3]
            t.transform.rotation = o.pose.pose.orientation
            self._tf_br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ZedBaseOdomRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
