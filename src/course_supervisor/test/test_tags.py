import math

import numpy as np
import pytest

from course_supervisor.tags import camera_optical_matrix, facing_of, robot_fix_from_tag, split_label, tag_matrix

cv2 = pytest.importorskip('cv2')

S = 0.16 / 2
OBJ = np.array([[-S, S, 0], [S, S, 0], [S, -S, 0], [-S, -S, 0]], np.float32)  # zed_apriltag corner model
K = np.array([[530, 0, 640], [0, 530, 360], [0, 0, 1.0]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def test_label_parsing():
    assert split_label('TUNNEL_EXIT @ 7.45, -6.10, 0.25, 88', 0.3) == ('TUNNEL_EXIT', (7.45, -6.1, 0.25, 88.0))
    assert split_label('TUNNEL_EXIT@7.45,-6.10,88', 0.3) == ('TUNNEL_EXIT', (7.45, -6.1, 0.3, 88.0))
    assert split_label('RampDetection', 0.3) == ('RampDetection', None)


def test_camera_optical_matrix_looks_forward():
    T = camera_optical_matrix([0.21, 0.08, 0.24], [0.0, 0.0, 0.0])
    assert np.allclose(T[:3, 2], [1, 0, 0])     # optical z = robot forward
    assert np.allclose(T[:3, 0], [0, -1, 0])    # optical x = robot right
    assert np.allclose(T[:3, 1], [0, 0, -1])    # optical y = down


def test_facing_roundtrip():
    for f in (-170.0, -92.0, 0.0, 45.0, 88.0, 179.0):
        assert abs(facing_of(tag_matrix(1, 2, 0.3, f)) - f) < 1e-6


def observe(x, y, yaw_deg, T_map_tag, noise_px, rng):
    """Simulate zed_apriltag: project the tag, solvePnP, return T_base_tag (4x4) and T_base_opt."""
    T_base_opt = np.eye(4)
    T_base_opt[:3, :3] = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0.0]])  # optical: z fwd, x right, y down
    T_base_opt[:3, 3] = [0.30, 0.0, 0.25]
    T_map_base = np.eye(4)
    T_map_base[:3, :3] = rz(math.radians(yaw_deg))
    T_map_base[:3, 3] = [x, y, 0.0]
    T_opt_tag = np.linalg.inv(T_map_base @ T_base_opt) @ T_map_tag
    pts = (T_opt_tag[:3, :3] @ OBJ.T).T + T_opt_tag[:3, 3]
    uv = (K @ pts.T).T
    uv = uv[:, :2] / uv[:, 2:] + rng.normal(0, noise_px, (4, 2))
    ok, rvec, tvec = cv2.solvePnP(OBJ, uv.astype(np.float32), K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    assert ok
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.ravel()
    return T_base_opt @ T


CASES = [(7.24, -4.8, -92), (7.40, -5.0, -85), (7.10, -4.6, -100), (7.30, -5.3, -92), (7.0, -4.3, -80)]


@pytest.mark.parametrize('noise_px', [0.0, 0.3, 1.0])
@pytest.mark.parametrize('x,y,yaw_deg', CASES)
def test_fix_is_robust_to_pnp_flip(x, y, yaw_deg, noise_px):
    # Tunnel-exit-like tag ahead near the right wall, face pointing back at the robot.
    T_map_tag = tag_matrix(7.45, -6.10, 0.25, 88.0)
    rng = np.random.default_rng(1)
    T_base_tag = observe(x, y, yaw_deg, T_map_tag, noise_px, rng)
    if not np.all(np.isfinite(T_base_tag)):
        pytest.skip('degenerate PnP (NaN) on noise-free input; the supervisor drops non-finite tag poses')
    prior_err = math.radians(2.0)                    # current heading estimate is 2 deg off
    fx, fy, fyaw, from_tag = robot_fix_from_tag(T_map_tag, T_base_tag, math.radians(yaw_deg) + prior_err,
                                                math.radians(8.0))
    dist = float(np.linalg.norm(T_base_tag[:2, 3]))
    pos_err = math.hypot(fx - x, fy - y)
    yaw_err = abs(math.degrees((fyaw - math.radians(yaw_deg) + math.pi) % (2 * math.pi) - math.pi))
    # Position error bounded by range * heading error (+ measurement noise), never metres.
    assert pos_err < dist * math.tan(prior_err) + 0.03, f'pos err {pos_err:.3f} m'
    assert yaw_err <= 8.0 + 2.0
