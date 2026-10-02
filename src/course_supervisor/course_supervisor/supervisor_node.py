#!/usr/bin/env python3
"""
course_supervisor -- picks which controller drives, section by section.

Every controller publishes to its own topic; this node forwards only the active
one, so switching is a topic choice and no controller has to be restarted. This
node is the ONLY publisher on `drive_cmd_topic` (/cmd_vel_smoothed, what
ackermann-drive listens to):
  nav2 / path_tracker: selected command -> `output_cmd_topic` (velocity smoother
      input) -> smoother -> `smoother_out_topic` -> forwarded to the drive.
  wall_follower: its command goes straight to the drive (as when it runs
      standalone on /cmd_vel_smoothed, keeping its own rate limits and reverse
      recovery, which the forward-only smoother would block). It is also fed to
      the smoother so the smoother's state matches when another mode takes over.

  nav2           Nav2 controller_server follows the course driving line via the
                 FollowPath action (no planner / global costmap). Its output
                 (`nav2_cmd_topic`) is forwarded.
  wall_follower  mapless_wall_follower is enabled and its output
                 (`wall_cmd_topic`) is forwarded; it is disabled elsewhere.
  path_tracker   built-in pure pursuit on the driving line using the map pose;
                 ignores the costmap (push through).
  stop           zero velocity.

The active section comes from monotonic progress along the driving line (see
course.Progress). Optional extras, each behind its own parameter:
  * drive_lidar_gate: publish mcl_3dl's `mcl_measurement_enabled` from the
    section's `lidar` flag (needs mcl_3dl use_measurement_gate:=true).
  * use_apriltag_fix: AprilTag sightings with a known map pose become pose
    measurements on `mcl_measurement` for mcl_3dl.
  * use_tag_triggers: a section's `trigger_label` (label from zed_apriltag's
    tag_labels.yaml) or `trigger_tag` (ID) seen within `trigger_range`
    advances progress to that section (forward only, next two sections).
  * tag_survey_file: record the map pose of every tag seen while lidar
    localization is active, for building the tag map.
A mode that is switched off falls back to path_tracker, then nav2, then stop.
"""
import math
import os
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from course_supervisor.course import Course, Progress, pure_pursuit
from course_supervisor.tags import (camera_optical_matrix, entry_matrix, quat_to_mat, robot_fix_from_tag,
                                     split_label, tag_matrix)

try:
    from nav2_msgs.action import FollowPath
except ImportError:  # Nav2 not installed: nav2 mode unavailable
    FollowPath = None

try:
    from zed_apriltag.msg import TagInfo
except ImportError:  # AprilTag package not built: tag features unavailable
    TagInfo = None


def tf_to_mat(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.x, t.y, t.z]
    return T


class CourseSupervisor(Node):
    def __init__(self):
        super().__init__('course_supervisor')
        p = self.declare_parameter
        p('course_file', '')
        p('robot_frame', 'base_footprint')
        p('rate', 20.0)
        p('output_cmd_topic', 'cmd_vel_mppi_raw')
        p('smoother_out_topic', '/cmd_vel_smoother_out')
        p('drive_cmd_topic', '/cmd_vel_smoothed')
        p('cmd_timeout', 0.3)
        p('goal_tolerance', 0.3)
        p('laps', 1)
        p('progress_back', 0.5)
        p('progress_ahead', 2.5)
        p('max_path_offset', 1.5)
        p('wait_for_green_light', True)
        p('green_light_topic', '/green_light')
        p('use_nav2', True)
        p('nav2_cmd_topic', '/cmd_vel_mppi')
        p('follow_path_action', 'follow_path')
        p('controller_id', 'FollowPath')
        p('goal_checker_id', 'general_goal_checker')
        p('nav2_preload_distance', 3.0)
        p('nav2_resend_period', 2.0)
        p('use_wall_follower', True)
        p('wall_cmd_topic', '/cmd_vel_wall_follower')
        p('wall_enable_topic', '/mapless_wall_follower/enable')
        p('use_path_tracker', True)
        p('tracker_speed', 0.30)
        p('tracker_lookahead', 0.6)
        p('min_turn_radius', 0.371)
        p('drive_lidar_gate', False)
        p('lidar_gate_topic', 'mcl_measurement_enabled')
        p('use_apriltag_fix', False)
        p('use_tag_triggers', False)
        p('tag_info_topic', 'apriltag/tag_info')
        p('tag_frame_prefix', 'tag_')
        p('tag_map_file', '')
        p('tag_survey_file', '')
        p('tag_max_range', 2.0)
        p('tag_sigma_xy', 0.05)
        p('tag_sigma_yaw_deg', 3.0)
        p('tag_min_interval', 0.5)
        p('measurement_topic', 'mcl_measurement')
        p('use_label_poses', True)
        p('tag_default_z', 0.25)
        p('tag_yaw_gate_deg', 8.0)
        # If TF has no robot_frame -> <camera optical frame> chain, use this camera mount
        # (robot_frame -> left camera, rpy 0 = looking forward) with the detector's own
        # <optical frame> -> tag_<id> transform.
        p('camera_tf_fallback', True)
        p('camera_offset_xyz', [0.21, 0.08, 0.24])
        p('camera_offset_rpy', [0.0, 0.0, 0.0])
        p('tag_prior_yaw_sigma_deg', 3.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        course_file = g('course_file')
        if not course_file:
            raise RuntimeError('parameter course_file is required')
        self.course = Course.load(course_file)
        self.progress = Progress(self.course, g('progress_back'), g('progress_ahead'), g('max_path_offset'))
        self.robot_frame = g('robot_frame')
        self.cmd_timeout = float(g('cmd_timeout'))
        self.goal_tolerance = float(g('goal_tolerance'))
        self.laps = max(1, int(g('laps')))
        self.lap = 1
        self.tracker_speed = float(g('tracker_speed'))
        self.tracker_lookahead = float(g('tracker_lookahead'))
        self.min_turn_radius = float(g('min_turn_radius'))
        self.preload = float(g('nav2_preload_distance'))
        self.resend_period = float(g('nav2_resend_period'))
        self.controller_id = g('controller_id')
        self.goal_checker_id = g('goal_checker_id')

        self.available = {
            'nav2': bool(g('use_nav2')) and FollowPath is not None,
            'wall_follower': bool(g('use_wall_follower')),
            'path_tracker': bool(g('use_path_tracker')),
            'stop': True,
        }
        if g('use_nav2') and FollowPath is None:
            self.get_logger().error('nav2_msgs not found: nav2 mode disabled')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, g('output_cmd_topic'), 10)
        self.drive_pub = self.create_publisher(Twist, g('drive_cmd_topic'), 10)
        self.create_subscription(Twist, g('smoother_out_topic'), self.on_smoother_out, 10)
        self.state_pub = self.create_publisher(String, '~/state', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '~/course_path', latched)
        self.path_pub.publish(self.make_path(0.0, next_lap=False))

        self.latest = {}
        if self.available['nav2']:
            self.create_subscription(Twist, g('nav2_cmd_topic'), lambda m: self.store('nav2', m), 10)
            self.nav2_client = ActionClient(self, FollowPath, g('follow_path_action'))
        self.nav2_goal = None
        self.nav2_pending = False
        self.nav2_last_send = 0.0
        if self.available['wall_follower']:
            self.create_subscription(Twist, g('wall_cmd_topic'), lambda m: self.store('wall_follower', m), 10)
            self.wall_enable_pub = self.create_publisher(Bool, g('wall_enable_topic'), 10)
        self.wall_enabled = None
        self.wall_enable_last = 0.0

        self.drive_lidar_gate = bool(g('drive_lidar_gate'))
        if self.drive_lidar_gate:
            self.gate_pub = self.create_publisher(Bool, g('lidar_gate_topic'), 10)

        self.started = not bool(g('wait_for_green_light'))
        if not self.started:
            self.create_subscription(Bool, g('green_light_topic'), self.on_green_light, 10)

        self.use_tag_fix = bool(g('use_apriltag_fix'))
        self.use_tag_triggers = bool(g('use_tag_triggers'))
        self.tag_survey_file = g('tag_survey_file')
        self.tag_prefix = g('tag_frame_prefix')
        self.tag_max_range = float(g('tag_max_range'))
        self.tag_sigma_xy = float(g('tag_sigma_xy'))
        self.tag_sigma_yaw = math.radians(float(g('tag_sigma_yaw_deg')))
        self.tag_min_interval = float(g('tag_min_interval'))
        self.tag_last = {}
        self.use_label_poses = bool(g('use_label_poses'))
        self.tag_default_z = float(g('tag_default_z'))
        self.tag_yaw_gate = math.radians(float(g('tag_yaw_gate_deg')))
        self.camera_tf_fallback = bool(g('camera_tf_fallback'))
        self.T_base_optical = camera_optical_matrix([float(v) for v in g('camera_offset_xyz')],
                                                    [float(v) for v in g('camera_offset_rpy')])
        self.camera_fallback_warned = False
        self.tag_prior_yaw_sigma = math.radians(float(g('tag_prior_yaw_sigma_deg')))
        self.tag_map = self.load_tag_map(g('tag_map_file')) if g('tag_map_file') else {}
        self.survey = {}
        self.survey_last_write = 0.0
        if self.use_tag_fix or self.use_tag_triggers or self.tag_survey_file:
            if TagInfo is None:
                self.get_logger().error('zed_apriltag messages not found: AprilTag features disabled')
            else:
                self.create_subscription(TagInfo, g('tag_info_topic'), self.on_tag, 10)
                if self.use_tag_fix:
                    self.meas_pub = self.create_publisher(PoseWithCovarianceStamped, g('measurement_topic'), 10)
                    if not self.tag_map:
                        self.get_logger().warn('use_apriltag_fix is on but tag_map_file has no tags')

        self.section_idx = -1
        self.mode = 'stop'
        self.finished = False
        self.last_state_pub = 0.0
        self.create_timer(1.0 / float(g('rate')), self.tick)
        modes = [m for m, ok in self.available.items() if ok and m != 'stop']
        self.get_logger().info(
            f'course {course_file}: {self.course.length:.1f} m, {len(self.course.sections)} sections; '
            f'modes available: {modes}; lidar gate: {self.drive_lidar_gate}; '
            f'tag fix: {self.use_tag_fix}, tag triggers: {self.use_tag_triggers}; '
            f'{"waiting for green light" if not self.started else "starting now"}')

    # ------------------------------------------------------------------ inputs
    def store(self, key, msg):
        self.latest[key] = (msg, time.monotonic())

    def on_smoother_out(self, msg):
        # Smoother output reaches the drive in every mode except wall_follower.
        if self.mode != 'wall_follower':
            self.drive_pub.publish(msg)

    def on_green_light(self, msg):
        if msg.data and not self.started:
            self.started = True
            self.get_logger().info('green light: go')

    # ---------------------------------------------------------------- helpers
    def resolve_mode(self, wanted):
        for m in (wanted, 'path_tracker', 'nav2', 'stop'):
            if self.available.get(m):
                return m
        return 'stop'

    def robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.course.frame_id, self.robot_frame, Time())
        except TransformException:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def make_path(self, s_from, next_lap=None):
        """Course line from s_from to the end; with laps remaining, the next lap follows on
        (the line ends next to where it starts), so Nav2 doesn't stop at the finish."""
        msg = Path()
        msg.header.frame_id = self.course.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        if next_lap is None:
            next_lap = self.lap < self.laps
        i0 = int(np.searchsorted(self.course.s, max(s_from, 0.0)))
        idx = list(range(max(i0 - 1, 0), len(self.course.s)))
        if next_lap:
            idx += list(range(1, len(self.course.s)))
        for i in idx:
            ps = PoseStamped()
            ps.header.frame_id = self.course.frame_id
            ps.pose.position.x = float(self.course.xy[i, 0])
            ps.pose.position.y = float(self.course.xy[i, 1])
            yaw = float(self.course.yaw[i])
            ps.pose.orientation.z = math.sin(yaw / 2)
            ps.pose.orientation.w = math.cos(yaw / 2)
            msg.poses.append(ps)
        return msg

    def publish_cmd(self, v=0.0, w=0.0, msg=None):
        out = msg if msg is not None else Twist()
        if msg is None:
            out.linear.x, out.angular.z = float(v), float(w)
        self.cmd_pub.publish(out)

    def forward(self, key):
        item = self.latest.get(key)
        msg = Twist() if item is None or time.monotonic() - item[1] > self.cmd_timeout else item[0]
        self.publish_cmd(msg=msg)
        return msg

    def set_wall_enabled(self, on, force=False):
        if not self.available['wall_follower']:
            return
        now = time.monotonic()
        if force or on != self.wall_enabled or now - self.wall_enable_last > 1.0:
            self.wall_enable_pub.publish(Bool(data=bool(on)))
            self.wall_enabled = on
            self.wall_enable_last = now

    # -------------------------------------------------------------- nav2 goal
    def nav2_wanted(self, s):
        if not self.available['nav2'] or self.finished:
            return False
        idx = self.course.section_index(s)
        if self.resolve_mode(self.course.sections[idx].mode) == 'nav2':
            return True
        nxt = self.course.section_index(s + self.preload)
        return any(self.resolve_mode(self.course.sections[i].mode) == 'nav2' for i in range(idx + 1, nxt + 1))

    def manage_nav2(self, s):
        if not self.available['nav2']:
            return
        wanted = self.nav2_wanted(s)
        if not wanted:
            if self.nav2_goal is not None:
                self.nav2_goal.cancel_goal_async()
                self.nav2_goal = None
            return
        if self.nav2_goal is not None or self.nav2_pending:
            return
        now = time.monotonic()
        if now - self.nav2_last_send < self.resend_period:
            return
        if not self.nav2_client.server_is_ready():
            if now - self.nav2_last_send > 5.0:
                self.get_logger().warn('FollowPath action server not available yet')
                self.nav2_last_send = now
            return
        goal = FollowPath.Goal()
        goal.path = self.make_path(s - 0.3)
        goal.controller_id = self.controller_id
        goal.goal_checker_id = self.goal_checker_id
        self.nav2_pending = True
        self.nav2_last_send = now
        self.nav2_client.send_goal_async(goal).add_done_callback(self.on_nav2_goal)

    def on_nav2_goal(self, fut):
        self.nav2_pending = False
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('FollowPath goal rejected')
            return
        self.nav2_goal = handle
        handle.get_result_async().add_done_callback(lambda f, h=handle: self.on_nav2_result(h, f))

    def on_nav2_result(self, handle, fut):
        if self.nav2_goal is handle:
            self.nav2_goal = None
            status = fut.result().status if fut.result() is not None else -1
            if not self.finished:
                self.get_logger().info(f'FollowPath ended (status {status}); will resend while nav2 is needed')

    # ---------------------------------------------------------------- apriltag
    def load_tag_map(self, path):
        if not os.path.exists(path):
            self.get_logger().warn(f'tag map {path} not found')
            return {}
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        tags = {}
        for k, v in (data.get('tags') or {}).items():
            tags[int(k)] = entry_matrix(v, self.tag_default_z)
        self.get_logger().info(f'loaded {len(tags)} tag poses from {path}')
        return tags

    def on_tag(self, msg):
        now = time.monotonic()
        if now - self.tag_last.get(msg.id, 0.0) < self.tag_min_interval:
            return
        tag_frame = f'{self.tag_prefix}{msg.id}'
        stamp = Time.from_msg(msg.header.stamp)
        try:
            T_base_tag = tf_to_mat(self.tf_buffer.lookup_transform(
                self.robot_frame, tag_frame, stamp, Duration(seconds=0.05)))
        except TransformException as e:
            if not self.camera_tf_fallback:
                self.get_logger().warn(f'tag {msg.id}: no TF {self.robot_frame} -> {tag_frame} ({e})',
                                       throttle_duration_sec=5.0)
                return
            try:
                T_opt_tag = tf_to_mat(self.tf_buffer.lookup_transform(
                    msg.header.frame_id, tag_frame, stamp, Duration(seconds=0.05)))
            except TransformException as e2:
                self.get_logger().warn(f'tag {msg.id}: no TF {msg.header.frame_id} -> {tag_frame} ({e2})',
                                       throttle_duration_sec=5.0)
                return
            if not self.camera_fallback_warned:
                self.get_logger().warn(
                    f'no TF {self.robot_frame} -> {msg.header.frame_id}: using camera_offset_xyz/rpy '
                    f'for the camera mount')
                self.camera_fallback_warned = True
            T_base_tag = self.T_base_optical @ T_opt_tag
        if not np.all(np.isfinite(T_base_tag)):
            return                                   # degenerate PnP solution
        self.tag_last[msg.id] = now
        name, label_pose = split_label(msg.label, self.tag_default_z)
        rng = float(np.linalg.norm(T_base_tag[:3, 3]))
        if rng > self.tag_max_range:
            return

        if self.use_tag_triggers and self.started:
            idx = self.course.section_index(self.progress.s)
            for j in range(idx + 1, min(idx + 3, len(self.course.sections))):
                sec = self.course.sections[j]
                match = ((sec.trigger_tag is not None and int(sec.trigger_tag) == msg.id) or
                         (sec.trigger_label is not None and str(sec.trigger_label) == name))
                if match and rng <= sec.trigger_range:
                    self.progress.jump_to(sec.s_start)
                    self.get_logger().info(f'tag {msg.id} ({name}) at {rng:.2f} m: progress -> section {sec.name}')

        # Tag map file first; otherwise a pose written in the label ('NAME @ x, y, [z,] facing_deg').
        T_map_tag = self.tag_map.get(msg.id)
        if T_map_tag is None and self.use_label_poses and label_pose is not None:
            T_map_tag = tag_matrix(*label_pose)
        need_pose = (self.use_tag_fix and T_map_tag is not None) or self.tag_survey_file
        if not need_pose:
            return
        try:
            T_map_base = tf_to_mat(self.tf_buffer.lookup_transform(
                self.course.frame_id, self.robot_frame, Time.from_msg(msg.header.stamp), Duration(seconds=0.05)))
        except TransformException:
            return
        yaw_prior = math.atan2(T_map_base[1, 0], T_map_base[0, 0])

        if self.use_tag_fix and T_map_tag is not None:
            x, y, yaw, yaw_from_tag = robot_fix_from_tag(T_map_tag, T_base_tag, yaw_prior, self.tag_yaw_gate)
            sigma_yaw = self.tag_sigma_yaw if yaw_from_tag else self.tag_prior_yaw_sigma
            # Heading error moves the position fix by range * heading error.
            var_xy = self.tag_sigma_xy ** 2 + (rng * sigma_yaw) ** 2
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = self.course.frame_id
            pose.pose.pose.position.x, pose.pose.pose.position.y = float(x), float(y)
            pose.pose.pose.position.z = float(T_map_base[2, 3])
            pose.pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.pose.orientation.w = math.cos(yaw / 2)
            cov = [0.0] * 36
            cov[0] = cov[7] = var_xy
            cov[14] = cov[21] = cov[28] = 100.0     # z / roll / pitch: not constrained by the tag
            cov[35] = self.tag_sigma_yaw ** 2 if yaw_from_tag else 100.0
            pose.pose.covariance = cov
            self.meas_pub.publish(pose)
            self.get_logger().info(
                f'tag {msg.id} ({name}) fix: x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f} deg '
                f'({"heading from tag" if yaw_from_tag else "position only"}, range {rng:.2f} m)',
                throttle_duration_sec=1.0)

        if self.tag_survey_file:
            sec = self.course.sections[self.course.section_index(self.progress.s)]
            if sec.lidar and not self.progress.lost:
                T = T_map_base @ T_base_tag
                acc = self.survey.setdefault(msg.id, {'p': np.zeros(3), 'facing': [], 'n': 0})
                acc['p'] += T[:3, 3]
                acc['facing'].append(math.degrees(math.atan2(T[1, 2], T[0, 2])))
                acc['n'] += 1
                if now - self.survey_last_write > 5.0:
                    self.write_survey()

    def write_survey(self):
        tags = {}
        for tid, acc in self.survey.items():
            x, y, z = (float(v) for v in acc['p'] / acc['n'])
            # Median (on the circle around the first sample) is robust to occasional PnP flips.
            ref = acc['facing'][0]
            facing = ref + float(np.median([(f - ref + 180.0) % 360.0 - 180.0 for f in acc['facing']]))
            facing = (facing + 180.0) % 360.0 - 180.0
            tags[int(tid)] = {'x': round(x, 3), 'y': round(y, 3), 'z': round(z, 3),
                              'facing_deg': round(facing, 1), 'samples': int(acc['n'])}
        with open(self.tag_survey_file, 'w') as f:
            f.write('# Tag poses in the map frame, recorded by course_supervisor (tag_survey_file).\n')
            f.write('# facing_deg = map yaw the printed face points to. Copy into tag_map_obstacle_course.yaml.\n')
            f.write('tags:\n')
            for tid in sorted(tags):
                f.write(f'  {tid}: ' + yaml.safe_dump(tags[tid], default_flow_style=True, sort_keys=False).strip() + '\n')
        self.survey_last_write = time.monotonic()

    # -------------------------------------------------------------------- loop
    def tick(self):
        pose = self.robot_pose()
        if self.drive_lidar_gate:
            sec_now = self.course.sections[max(self.section_idx, 0)]
            self.gate_pub.publish(Bool(data=bool(sec_now.lidar) if self.started and not self.finished else True))

        if not self.started or self.finished:
            self.publish_cmd()
            self.set_wall_enabled(False)
            self.publish_state(pose)
            return
        if pose is None:
            self.publish_cmd()
            self.get_logger().warn(f'no TF {self.course.frame_id} -> {self.robot_frame}; holding still',
                                   throttle_duration_sec=2.0)
            return

        x, y, yaw = pose
        s = self.progress.update(x, y)
        if s >= self.course.length - self.goal_tolerance and self.lap < self.laps:
            self.lap += 1
            self.progress.reset()
            self.get_logger().info(f'lap {self.lap - 1} done: starting lap {self.lap}/{self.laps}')
            s = self.progress.update(x, y)
        if s >= self.course.length - self.goal_tolerance:
            self.finished = True
            self.manage_nav2(s)
            self.publish_cmd()
            self.set_wall_enabled(False, force=True)
            self.get_logger().info(f'course finished ({self.laps} lap(s)): stopped')
            return

        idx = self.course.section_index(s)
        sec = self.course.sections[idx]
        mode = self.resolve_mode(sec.mode)
        if idx != self.section_idx or mode != self.mode:
            self.get_logger().info(f'lap {self.lap}/{self.laps} section {sec.name} (s={s:.2f} m): mode {mode}'
                                   + (f' (wanted {sec.mode})' if mode != sec.mode else '')
                                   + ('' if sec.lidar else ', lidar off'))
            self.section_idx, self.mode = idx, mode
        self.set_wall_enabled(mode == 'wall_follower')
        self.manage_nav2(s)

        if mode == 'nav2':
            self.forward('nav2')
        elif mode == 'wall_follower':
            self.drive_pub.publish(self.forward('wall_follower'))
        elif mode == 'path_tracker':
            if self.progress.lost:
                self.publish_cmd()
                self.get_logger().warn(f'{self.progress.offset:.2f} m off the driving line; path tracker stopped',
                                       throttle_duration_sec=2.0)
            else:
                speed = sec.speed if sec.speed is not None else self.tracker_speed
                v, w = pure_pursuit(self.course, x, y, yaw, s, speed, self.tracker_lookahead, self.min_turn_radius)
                self.publish_cmd(v, w)
        else:
            self.publish_cmd()
        self.publish_state(pose)

    def publish_state(self, pose):
        now = time.monotonic()
        if now - self.last_state_pub < 0.5:
            return
        self.last_state_pub = now
        sec = self.course.sections[max(self.section_idx, 0)]
        status = 'finished' if self.finished else ('running' if self.started else 'waiting_for_green_light')
        self.state_pub.publish(String(data=(
            f'{status} lap={self.lap}/{self.laps} section={sec.name} mode={self.mode} '
            f's={self.progress.s:.2f}/{self.course.length:.2f} '
            f'offset={self.progress.offset:.2f} pose={"none" if pose is None else "ok"}')))

    def shutdown(self):
        if self.tag_survey_file and self.survey:
            self.write_survey()
        try:
            self.publish_cmd()
            self.drive_pub.publish(Twist())
            self.set_wall_enabled(False, force=True)
        except Exception:  # noqa: BLE001 - context may already be gone
            pass


def main():
    rclpy.init()
    node = CourseSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
