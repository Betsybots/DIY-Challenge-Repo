"""AprilTag geometry for course_supervisor.

Tag frame (zed_apriltag, solvePnP with corners (-s,s), (s,s), (s,-s), (-s,-s)):
x right as seen when facing the tag, y up, z out of the printed face (toward
the camera). A tag mounted upright is fully described in the map by its centre
(x, y, z) and `facing_deg`: the map yaw its printed face points to.
"""
import math
import re

import numpy as np

_LABEL_POSE = re.compile(
    r'^\s*(?P<name>[^@]*?)\s*@\s*(?P<nums>-?\d+(?:\.\d*)?(?:\s*,\s*-?\d+(?:\.\d*)?){2,3})\s*$')


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return tuple(q)


def pose_to_mat(position, orientation):
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(*orientation)
    T[:3, 3] = position
    return T


# Optical frame (z forward, x right, y down) expressed in a camera body frame (x forward, y left, z up).
R_BODY_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def camera_optical_matrix(xyz, rpy):
    """4x4 robot_frame -> camera optical frame from the camera mount (xyz in m, rpy in rad;
    rpy = 0 means the camera looks along the robot's +x)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                  [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                  [-sp, cp * sr, cp * cr]])
    T = np.eye(4)
    T[:3, :3] = R @ R_BODY_OPTICAL
    T[:3, 3] = xyz
    return T


def tag_matrix(x, y, z, facing_deg):
    """4x4 pose of an upright tag in the map."""
    f = math.radians(facing_deg)
    zt = np.array([math.cos(f), math.sin(f), 0.0])     # out of the face
    yt = np.array([0.0, 0.0, 1.0])                      # up
    xt = np.cross(yt, zt)                               # right, as seen facing the tag
    T = np.eye(4)
    T[:3, :3] = np.column_stack([xt, yt, zt])
    T[:3, 3] = [x, y, z]
    return T


def facing_of(T):
    """Map yaw the tag face points to (from its z axis)."""
    return math.degrees(math.atan2(T[1, 2], T[0, 2]))


def split_label(label, default_z):
    """'NAME @ x, y, [z,] facing_deg' -> (name, (x, y, z, facing) or None)."""
    m = _LABEL_POSE.match(label or '')
    if not m:
        return (label or '').strip(), None
    nums = [float(v) for v in m.group('nums').split(',')]
    if len(nums) == 3:
        nums = [nums[0], nums[1], default_z, nums[2]]
    return m.group('name').strip(), tuple(nums)


def entry_matrix(entry, default_z):
    """Tag-map entry -> 4x4. Accepts {x, y, [z,] facing_deg} or {position, orientation}."""
    if 'facing_deg' in entry:
        return tag_matrix(float(entry['x']), float(entry['y']), float(entry.get('z', default_z)),
                          float(entry['facing_deg']))
    return pose_to_mat(entry['position'], entry['orientation'])


def robot_from_tag(T_map_tag, T_robot_tag):
    """Robot pose in the map from a tag's full 6-DoF pose seen from the robot.

    Uses the PnP rotation, which is ambiguous when a tag is seen nearly
    face-on (IPPE "flip"): a flipped rotation moves this estimate by metres.
    Prefer robot_fix_from_tag for fixes.
    """
    return T_map_tag @ np.linalg.inv(T_robot_tag)


def yaw_of(T):
    return math.atan2(T[1, 0], T[0, 0])


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def robot_fix_from_tag(T_map_tag, T_robot_tag, yaw_prior, yaw_gate):
    """Robust 2D robot fix (x, y, yaw, yaw_from_tag) from one tag sighting.

    Position uses only the tag's measured position in the robot frame (depth and
    sideways offset; unaffected by the PnP rotation flip) together with a
    heading. The heading is the tag-derived one when it agrees with `yaw_prior`
    (current localization) within `yaw_gate` rad, otherwise `yaw_prior`, and
    then the fix carries no heading information (yaw_from_tag False).
    """
    yaw_tag = yaw_of(robot_from_tag(T_map_tag, T_robot_tag))
    use_tag_yaw = abs(wrap(yaw_tag - yaw_prior)) <= yaw_gate
    yaw = yaw_tag if use_tag_yaw else yaw_prior
    m = T_robot_tag[:2, 3]                      # tag centre in the robot frame (x fwd, y left)
    c, s = math.cos(yaw), math.sin(yaw)
    x = T_map_tag[0, 3] - (c * m[0] - s * m[1])
    y = T_map_tag[1, 3] - (s * m[0] + c * m[1])
    return x, y, yaw, use_tag_yaw
