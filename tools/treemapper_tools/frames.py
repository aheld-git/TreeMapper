"""Per-frame INPUT contract of the mapping system, plus its file form and the Isaac bag reader.

One ``Frame`` is everything the system receives for one camera instant (brief section 4):

* ``t``          receive (bag) time, the clock shared with ``mission.json`` and the map stream;
* ``stamp``      image stamp (detections and disparity are paired on it exactly);
* ``pose``       drone (``body``) pose in the world frame at ``stamp``: x, y, z, qx, qy, qz, qw;
                 ``pose_valid`` false = unknown (the system must hold processing, brief section 8);
* ``phase``      mission phase string as navigation sends it ("" = no information);
* ``target_id``  navigation's hop target ID **in navigation's own ID space** (-1 = none). In a replay
                 this is the ID of the recorded (existing) system, which means nothing to another map;
* ``target_xy``  where navigation is going to descend for that target (NaN = unknown). The replay's
                 navigation emulator (``runner``) resolves it to the mapper's own ID;
* ``boxes``      (N, 6) float32: x, y, w, h (pixels, top-left corner, full image), score, label;
* ``disparity``  (gh, gw) float32 disparity in **full-resolution pixels**, sampled on a regular grid
                 (pixel (r, c) of the grid = image pixel (v, u) = (r*s + o, c*s + o), s = stride,
                 o = stride // 2); NaN = invalid. ``None`` = no disparity for this frame (heartbeat
                 frame while the detector sleeps, or no disparity with that stamp).

``Calibration`` (once per stream): image size, left rectified intrinsics fx, fy, cx, cy, stereo baseline
(m), the grid stride, and ``T_body_cam`` (translation + quaternion xyzw: optical camera frame -> body).
A world point is ``R_wb (R_bc p_opt + t_bc) + t_wb`` with ``p_opt = ((u-cx) Z/fx, (v-cy) Z/fy, Z)``,
``Z = fx * B / d``. Sim vs real conventions are handled entirely in the reader (a configuration).

File form (``frames.bin``, little endian), read by both Python and the C++ runner::

    b"TMF1" | u32 header_len | header JSON (calibration + meta) |
    15 x f64 calibration (width height fx fy cx cy baseline stride tx ty tz qx qy qz qw) |
    records: f64 t, f64 stamp, 7 x f64 pose, u8 pose_valid, i8 phase_code, i32 target_id,
             2 x f64 target_xy, u16 n_boxes, u8 has_disp, n_boxes x 6 x f32,
             [gh x gw x u16 disparity * 256 (0 = invalid)]

Disparity is cached on the grid only (stride 4 -> 160 x 100 = 32 kB per frame) -- the full 1 MB frames
are never written to disk. Quantisation 1/256 px is far below any stereo noise.
"""
from __future__ import annotations

import json
import math
import struct
from dataclasses import dataclass, field
from typing import Dict, Iterator, List, Optional, Sequence, Tuple

import numpy as np

MAGIC = b"TMF1"
PHASES = ["", "IDLE", "SCAN_APPROACH", "SCAN", "PLANNING", "HOPPING", "SPRAYING", "RETURN", "ERROR"]
PHASE_CODE = {p: i for i, p in enumerate(PHASES)}
_REC = struct.Struct("<dd7dBbi2dHB")
_CAL = struct.Struct("<15d")  # width height fx fy cx cy baseline stride tx ty tz qx qy qz qw
DISP_SCALE = 256.0


@dataclass
class Calibration:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    baseline: float
    stride: int = 4
    T_body_cam: List[float] = field(default_factory=lambda: [0, 0, 0, 0, 0, 0, 1])  # x y z qx qy qz qw
    frame: str = "world"
    note: str = ""

    @property
    def grid_shape(self) -> Tuple[int, int]:
        o = self.stride // 2
        return (len(range(o, self.height, self.stride)), len(range(o, self.width, self.stride)))

    def grid_uv(self) -> Tuple[np.ndarray, np.ndarray]:
        o = self.stride // 2
        v = np.arange(o, self.height, self.stride, dtype=np.float64)
        u = np.arange(o, self.width, self.stride, dtype=np.float64)
        return np.meshgrid(u, v)  # (gh, gw) each

    def to_json(self) -> dict:
        return {k: getattr(self, k) for k in
                ("width", "height", "fx", "fy", "cx", "cy", "baseline", "stride", "T_body_cam", "frame", "note")}

    @staticmethod
    def from_json(d: dict) -> "Calibration":
        return Calibration(**{k: d[k] for k in ("width", "height", "fx", "fy", "cx", "cy", "baseline",
                                                "stride", "T_body_cam", "frame", "note") if k in d})


@dataclass
class Frame:
    t: float
    stamp: float
    pose: Tuple[float, ...]  # x y z qx qy qz qw (world <- body)
    pose_valid: bool
    phase: str
    target_id: int
    target_xy: Tuple[float, float]
    boxes: np.ndarray  # (N, 6) float32
    disparity: Optional[np.ndarray]  # (gh, gw) float32, NaN invalid


# ----------------------------------------------------------------------------- geometry

def quat_to_rot(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def backproject(cal: Calibration, u: np.ndarray, v: np.ndarray, d: np.ndarray,
                pose: Sequence[float]) -> np.ndarray:
    """Pixels (u, v) with disparity d (full-res px) -> (N, 3) world points. d <= 0 / NaN -> NaN."""
    d = np.where(np.isfinite(d) & (d > 0), d, np.nan)
    Z = cal.fx * cal.baseline / d
    p = np.stack([(u - cal.cx) * Z / cal.fx, (v - cal.cy) * Z / cal.fy, Z], axis=-1)
    tb = np.asarray(cal.T_body_cam[:3]); Rb = quat_to_rot(cal.T_body_cam[3:])
    tw = np.asarray(pose[:3]); Rw = quat_to_rot(pose[3:7])
    return (p @ Rb.T + tb) @ Rw.T + tw


def interp_pose(ts: np.ndarray, rows: np.ndarray, t: float, max_gap: float = 0.25) -> Optional[Tuple[float, ...]]:
    """Linear position + normalised-lerp quaternion at t from sorted samples; None if out of range."""
    k = int(np.searchsorted(ts, t))
    if k == 0 or k >= len(ts):
        if len(ts) and abs(ts[min(k, len(ts) - 1)] - t) < 1e-6:
            return tuple(rows[min(k, len(ts) - 1)])
        return None
    t0, t1 = ts[k - 1], ts[k]
    if t1 - t0 > max_gap:
        return None
    a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
    p = rows[k - 1, :3] * (1 - a) + rows[k, :3] * a
    q0, q1 = rows[k - 1, 3:], rows[k, 3:]
    if np.dot(q0, q1) < 0:
        q1 = -q1
    q = q0 * (1 - a) + q1 * a
    q = q / np.linalg.norm(q)
    return tuple(p.tolist() + q.tolist())


# ----------------------------------------------------------------------------- file form

class FrameWriter:
    def __init__(self, path: str, cal: Calibration, meta: Optional[dict] = None):
        self.f = open(path, "wb")
        self.cal = cal
        hdr = json.dumps({"calibration": cal.to_json(), "meta": meta or {}}).encode()
        self.f.write(MAGIC + struct.pack("<I", len(hdr)) + hdr)
        # binary copy of the calibration for readers without JSON (the C++ runner)
        self.f.write(_CAL.pack(cal.width, cal.height, cal.fx, cal.fy, cal.cx, cal.cy, cal.baseline,
                               cal.stride, *[float(v) for v in cal.T_body_cam]))
        self.n = 0

    def write(self, fr: Frame) -> None:
        boxes = np.asarray(fr.boxes, dtype=np.float32).reshape(-1, 6)
        has = fr.disparity is not None
        self.f.write(_REC.pack(fr.t, fr.stamp, *[float(v) for v in fr.pose], int(fr.pose_valid),
                               PHASE_CODE.get(fr.phase, 0), int(fr.target_id),
                               float(fr.target_xy[0]), float(fr.target_xy[1]), len(boxes), int(has)))
        self.f.write(boxes.tobytes())
        if has:
            d = np.asarray(fr.disparity, dtype=np.float64)
            assert d.shape == self.cal.grid_shape, (d.shape, self.cal.grid_shape)
            q = np.where(np.isfinite(d) & (d > 0), np.clip(np.round(d * DISP_SCALE), 1, 65535), 0)
            self.f.write(q.astype("<u2").tobytes())
        self.n += 1

    def close(self) -> None:
        self.f.close()


def read_frames(path: str) -> Tuple[Calibration, dict, Iterator[Frame]]:
    f = open(path, "rb")
    if f.read(4) != MAGIC:
        raise ValueError("%s: not a TMF1 frame file" % path)
    (n,) = struct.unpack("<I", f.read(4))
    hdr = json.loads(f.read(n).decode())
    cal = Calibration.from_json(hdr["calibration"])
    f.read(_CAL.size)
    gh, gw = cal.grid_shape

    def gen():
        with f:
            while True:
                b = f.read(_REC.size)
                if len(b) < _REC.size:
                    return
                r = _REC.unpack(b)
                t, stamp, pose, valid, ph, tid, tx, ty, nb, has = r[0], r[1], r[2:9], r[9], r[10], r[11], r[12], r[13], r[14], r[15]
                boxes = np.frombuffer(f.read(nb * 24), dtype="<f4").reshape(nb, 6)
                disp = None
                if has:
                    q = np.frombuffer(f.read(gh * gw * 2), dtype="<u2").reshape(gh, gw).astype(np.float32)
                    disp = np.where(q > 0, q / DISP_SCALE, np.nan).astype(np.float32)
                yield Frame(t, stamp, tuple(pose), bool(valid), PHASES[ph] if 0 <= ph < len(PHASES) else "",
                            tid, (tx, ty), boxes, disp)
    return cal, hdr.get("meta", {}), gen()
