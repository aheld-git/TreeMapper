"""Frame and camera-rig math shared by the Isaac modules (pure numpy, no Isaac imports).

Conventions
    world   ENU, z up.
    body    FLU: x forward, y left, z up. Orientation from the controller state is
            R = Rz(yaw) Ry(pitch) Rx(roll) (pitch positive nose-down, roll positive left-side-up).
    optical x right, y down, z forward (ROS camera optical frame; what ``camera.json`` calls the camera).
    USD cam x right, y up, looking along -z. ``R_usd = R_optical * diag(1, -1, -1)``.

Quaternions are (x, y, z, w) like ROS.
"""
import json
import math
from typing import Dict, Sequence, Tuple

import numpy as np

# Half-pixel difference between the renderer's image coordinates and the OpenCV pixel indices of camera_info.
PIXEL_CENTRE_SHIFT = 0.5

# optical -> USD camera axes: y and z flipped
OPTICAL_TO_USD = np.diag([1.0, -1.0, -1.0])


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def body_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Body -> world rotation for the controller's roll / pitch / yaw."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def quat_to_rot(q: Sequence[float]) -> np.ndarray:
    """(x, y, z, w) -> 3x3 rotation matrix (the quaternion is normalised first)."""
    x, y, z, w = (float(c) for c in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError("zero quaternion")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    """3x3 rotation matrix -> unit quaternion (x, y, z, w), w >= 0."""
    R = np.asarray(R, dtype=float)
    t = float(np.trace(R))
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    return (x, y, z, w)


def rotate_cone(directions_body: np.ndarray, R_body_to_world: np.ndarray) -> np.ndarray:
    """Rows of ``directions_body`` (n x 3) expressed in the world frame."""
    return np.asarray(directions_body, dtype=float) @ np.asarray(R_body_to_world, dtype=float).T


def clock_skew_ok(graph_t: float, our_t: float, dt: float) -> bool:
    """True when the image time base (graph time) and our /clock agree within half a physics step.

    Written as ``abs(skew) <= dt / 2`` so a NaN on either side (or a NaN dt) is NOT ok: ``abs(nan) > x`` is False,
    which let a NaN graph time pass the old ``if abs(skew) > 0.5 * dt: fail`` guard.
    """
    skew = graph_t - our_t
    return bool(abs(skew) <= 0.5 * dt)


def split_stamp(t: float) -> Tuple[int, int]:
    """Seconds as float -> (sec, nanosec) with nanosec in [0, 1e9)."""
    sec = int(math.floor(t))
    nsec = int(round((t - sec) * 1e9))
    if nsec >= 1000000000:
        sec, nsec = sec + 1, nsec - 1000000000
    return sec, nsec


# ----------------------------------------------------------------------------- camera rig

class CameraRig:
    """The stereo rig of ``sim/specs/camera.json`` as the sim renders it.

    Both cameras are rendered as ideal rectified pinholes with the LEFT ``P`` intrinsics.
    The right camera is the left one translated +B along the left optical x axis.
    ``camera_info`` is published exactly as in the file (K, P, size; D zeros, plumb_bob).
    """

    def __init__(self, doc: Dict):
        left, right = doc["left"], doc["right"]
        self.width = int(left["width"])
        self.height = int(left["height"])
        if (int(right["width"]), int(right["height"])) != (self.width, self.height):
            raise ValueError("camera.json: left and right sizes differ")
        self.left_K = [float(v) for v in left["K"]]
        self.left_P = [float(v) for v in left["P"]]
        self.right_K = [float(v) for v in right["K"]]
        self.right_P = [float(v) for v in right["P"]]
        if len(self.left_K) != 9 or len(self.right_K) != 9 or len(self.left_P) != 12 or len(self.right_P) != 12:
            raise ValueError("camera.json: K must have 9 and P 12 numbers")
        self.fx, self.fy = self.left_P[0], self.left_P[5]
        self.cx, self.cy = self.left_P[2], self.left_P[6]
        # P_right[3] = -fx * B with B in millimetres (the driver's convention): -72724.8 / 454.53 = 160.0.
        tx = self.right_P[3]
        if not (tx < 0.0 and self.right_P[0] > 0.0):
            raise ValueError("camera.json: right P has no usable baseline (P[3] = %r)" % tx)
        self.baseline_m = -tx / self.right_P[0] / 1000.0
        m = [float(v) for v in doc["mount"]]
        if len(m) != 7:
            raise ValueError("camera.json: mount must be [x, y, z, qx, qy, qz, qw]")
        self.mount_t = np.array(m[:3])
        self.mount_R = quat_to_rot(m[3:])   # optical -> body
        self.mount_q = rot_to_quat(self.mount_R)

    def optical_pose_in_body(self, side: str) -> Tuple[np.ndarray, np.ndarray]:
        """(translation, rotation optical->body) of the ``left`` or ``right`` optical frame."""
        if side == "left":
            return self.mount_t.copy(), self.mount_R.copy()
        if side == "right":
            return self.mount_t + self.mount_R @ np.array([self.baseline_m, 0.0, 0.0]), self.mount_R.copy()
        raise ValueError(side)

    def usd_pose_in_body(self, side: str) -> Tuple[np.ndarray, np.ndarray]:
        """Pose of the USD camera prim (looks along -z) in the body frame."""
        t, R = self.optical_pose_in_body(side)
        return t, R @ OPTICAL_TO_USD

    def usd_camera_params(self, h_aperture_mm: float = 20.955) -> Dict[str, float]:
        """USD camera attributes that render an ideal pinhole with fx = fy and principal point (cx, cy).

        Measured with ``camera_check.py`` on Isaac Sim 5.1 (RaytracedLighting): the RTX renderer
        honours ``horizontalApertureOffset`` / ``verticalApertureOffset`` (mm, same units as the
        apertures); a NEGATIVE horizontal offset moves the principal point to +u and a POSITIVE
        vertical offset moves it to +v. The renderer puts the continuous image coordinate of the
        first pixel's centre at 0.5 (edge convention), OpenCV at 0.0, so the principal point is
        rendered 0.5 px further out (``PIXEL_CENTRE_SHIFT``) to land on (cx, cy) in OpenCV
        pixel indices, which is what the camera_info P describes.
        """
        v_ap = h_aperture_mm * self.height / self.width * (self.fx / self.fy)
        cx_r = self.cx + PIXEL_CENTRE_SHIFT
        cy_r = self.cy + PIXEL_CENTRE_SHIFT
        return {
            "focal_mm": self.fx * h_aperture_mm / self.width,
            "h_aperture_mm": h_aperture_mm,
            "v_aperture_mm": v_ap,
            "h_off_mm": -(cx_r - self.width / 2.0) * h_aperture_mm / self.width,
            "v_off_mm": (cy_r - self.height / 2.0) * v_ap / self.height,
        }

    def project(self, p_cam_optical: Sequence[float]) -> Tuple[float, float]:
        """Pixel (u, v) of a point in the left optical frame (ideal pinhole, left P)."""
        x, y, z = (float(c) for c in p_cam_optical)
        if z <= 0.0:
            raise ValueError("point is behind the camera")
        return self.fx * x / z + self.cx, self.fy * y / z + self.cy


def load_rig(path: str) -> CameraRig:
    with open(path) as f:
        return CameraRig(json.load(f))


def project_world_point(rig: CameraRig, body_t: Sequence[float], body_R: np.ndarray,
                        p_world: Sequence[float], side: str = "left") -> Tuple[float, float]:
    """Pixel of a world point seen by the ``left`` (default) or ``right`` camera for body pose (body_t, body_R).

    Both cameras are rendered with the left P intrinsics, so the right camera differs only by its pose.
    """
    t_opt, R_opt = rig.optical_pose_in_body(side)
    R_w_opt = np.asarray(body_R) @ R_opt
    t_w_opt = np.asarray(body_R) @ t_opt + np.asarray(body_t, dtype=float)
    p_cam = R_w_opt.T @ (np.asarray(p_world, dtype=float) - t_w_opt)
    return rig.project(p_cam)
