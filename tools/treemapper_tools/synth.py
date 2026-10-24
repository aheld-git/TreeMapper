"""Synthetic orchard generator (brief section 9): seeded, hard, same files as a recorded mission.

    python3 -m treemapper_tools.synth --out /data/work/synthetic/suite_v1 --levels easy,medium,hard --seeds 0-4 [-j 6]

Each dataset directory gets ``frames.bin`` (the per-frame input contract of ``frames.py``),
``ground_truth.json``, ``mission.json``, ``camera_info.json`` (the formats of ``contract.py``, as the
Isaac extraction writes them) and ``meta.json`` (seed, level, all parameters, counts). Same seed + level ->
byte-identical output.

World: rows of trees with varied radius, irregular (3-harmonic) outlines, dome canopies of varied height
and profile, leaf-scale surface roughness, missing trees, touching/overlapping canopies (level-dependent
spacing), ground slope + undulation, ground clutter (bushes) and grass texture.

Sensor: disparity is ray-cast from the TRUE camera pose against the world heightfield on the stride-4
grid (oblique rays see canopy sides; occlusion is geometric). Noise (all in disparity, so depth error
grows with range squared and lies along the line of sight): per-pixel Gaussian sigma_d, spatially
correlated low-frequency error, foreground fattening at depth edges, iid + blob dropouts, uniform
outliers, 1/16 px quantisation (like the real SGBM disparity). Corridor turns (yaw rate) raise every
noise term and degrade the detector (noise bursts). The system receives a NOISY pose (white noise +
random-walk bias; attitude noise).

Detector: projects each true canopy, detects it with level-dependent probability (sticky misses, worse
at the image border and in bursts), jitters/rescales boxes, merges boxes of touching trees, emits
partial (half) boxes, and false positives on clutter and bare ground.

Mission: IDLE (ground, take-off) -> SCAN_APPROACH -> SCAN (serpentine corridors at 15 m with U-turns)
-> PLANNING -> [HOPPING (transit 10.5 m, descent to 7 m) -> SPRAYING (4 s)] x n -> RETURN -> IDLE.
The detector sleeps in IDLE / SPRAYING / RETURN (heartbeat frames only), like the recorded mission.

Calibration of the noise against the recorded data: results/synthetic_calibration.md.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from multiprocessing import Pool
from typing import List, Optional

import numpy as np

from .contract import GroundTruthTree, Mission, write_ground_truth, write_mission
from .frames import Calibration, Frame, FrameWriter, quat_to_rot

# Camera = the recorded Isaac camera (half resolution), nadir mount derived in results/geometry_check.md.
CAM = dict(width=640, height=400, fx=454.53, fy=454.53, cx=346.71, cy=209.68, baseline=0.16)
BODY_CAM = [0.15, 0.08, 0.0, -0.70710678, 0.70710678, 0.0, 0.0]
STRIDE = 4

LEVELS = {
    #             in-row spacing (mean, sd) | canopy radius | top height | missing | ground slope | clutter /m2
    "easy": dict(spacing=(6.0, 0.4), row_spacing=6.5, radius=(1.9, 2.7), top=(3.5, 5.5), base=(0.8, 1.4),
                 shape_amp=0.06, missing=0.05, slope=0.00, undulation=0.10, clutter=0.010, rows=3, per_row=5,
                 speed=4.0, sigma_d=0.15, corr_d=0.05, drop=0.03, blob_drop=0.02, outlier=0.005, fatten=0.3,
                 burst=2.0, p_miss=0.08, miss_sticky=0.4, jitter=0.04, size_sd=0.06, fp=0.10, merge=0.15,
                 partial=0.03, pose_xy=0.03, pose_bias=0.01, pose_att_deg=0.10, n_spray=4),
    "medium": dict(spacing=(5.0, 0.5), row_spacing=6.0, radius=(1.9, 2.8), top=(3.0, 6.0), base=(0.6, 1.4),
                   shape_amp=0.09, missing=0.10, slope=0.03, undulation=0.25, clutter=0.025, rows=4, per_row=5,
                   speed=4.0, sigma_d=0.30, corr_d=0.10, drop=0.08, blob_drop=0.05, outlier=0.015, fatten=0.6,
                   burst=3.0, p_miss=0.15, miss_sticky=0.5, jitter=0.07, size_sd=0.08, fp=0.25, merge=0.35,
                   partial=0.07, pose_xy=0.05, pose_bias=0.02, pose_att_deg=0.20, n_spray=5),
    "hard": dict(spacing=(4.5, 0.5), row_spacing=5.6, radius=(1.8, 2.9), top=(2.5, 6.5), base=(0.5, 1.5),
                 shape_amp=0.12, missing=0.15, slope=0.06, undulation=0.40, clutter=0.045, rows=5, per_row=6,
                 speed=5.0, sigma_d=0.50, corr_d=0.20, drop=0.15, blob_drop=0.08, outlier=0.03, fatten=0.9,
                 burst=4.0, p_miss=0.25, miss_sticky=0.6, jitter=0.10, size_sd=0.10, fp=0.50, merge=0.60,
                 partial=0.12, pose_xy=0.10, pose_bias=0.04, pose_att_deg=0.40, n_spray=6),
}
FRAME_HZ = 15.0      # detector/disparity frame rate while awake (recorded: ~27 Hz)
HEARTBEAT_HZ = 5.0
RES = 0.05           # heightfield resolution, m
SCAN_ALT, HOP_ALT, SPRAY_ALT = 15.0, 10.5, 7.0
G = 9.81


# ----------------------------------------------------------------------------- world

class World:
    def __init__(self, rng: np.random.Generator, p: dict):
        self.p = p
        trees = []
        tid = 0
        for r in range(p["rows"]):
            x_row = r * p["row_spacing"]
            y = 0.0
            for k in range(p["per_row"]):
                if k:
                    y += max(2.5, rng.normal(*p["spacing"]))
                tid_here = tid
                tid += 1
                if rng.random() < p["missing"]:
                    continue
                trees.append(dict(id=tid_here, x=x_row + rng.uniform(-0.4, 0.4), y=y + rng.uniform(-0.3, 0.3),
                                  r=rng.uniform(*p["radius"]), top=rng.uniform(*p["top"]),
                                  base=rng.uniform(*p["base"]), prof=rng.uniform(0.6, 1.3),
                                  harm=[(rng.uniform(0, p["shape_amp"]), rng.uniform(0, 2 * math.pi)) for _ in range(3)]))
        self.trees = trees
        xs = [t["x"] for t in trees] or [0.0]; ys = [t["y"] for t in trees] or [0.0]
        m = 16.0
        self.x0, self.y0 = min(xs) - m, min(ys) - m
        nx = int((max(xs) + m - self.x0) / RES) + 1
        ny = int((max(ys) + m - self.y0) / RES) + 1
        self.nx, self.ny = nx, ny
        gx = self.x0 + RES * np.arange(nx); gy = self.y0 + RES * np.arange(ny)
        X, Y = np.meshgrid(gx, gy)  # (ny, nx)
        sa = rng.uniform(0, 2 * math.pi)
        self.slope = (p["slope"] * math.cos(sa), p["slope"] * math.sin(sa))
        und = [(rng.uniform(0.05, 0.2), rng.uniform(0, 2 * math.pi), rng.uniform(0, 2 * math.pi)) for _ in range(3)]
        self.und = und
        ground = self.ground_xy(X, Y)
        ground += rng.normal(0, 0.03, ground.shape)  # grass texture
        # clutter: bushes (gaussian bumps), kept off the canopies' centres
        area = (nx * RES) * (ny * RES)
        self.bushes = []
        for _ in range(rng.poisson(p["clutter"] * area)):
            bx, by = rng.uniform(self.x0, self.x0 + nx * RES), rng.uniform(self.y0, self.y0 + ny * RES)
            if any(math.hypot(bx - t["x"], by - t["y"]) < t["r"] * 0.8 for t in trees):
                continue
            self.bushes.append((bx, by, rng.uniform(0.3, 1.2), rng.uniform(0.2, 0.9)))
        h = ground.copy()
        for bx, by, br, bh in self.bushes:
            sl = self._window(bx, by, 3 * br)
            dx = X[sl] - bx; dy = Y[sl] - by
            h[sl] = np.maximum(h[sl], ground[sl] + bh * np.exp(-(dx * dx + dy * dy) / (2 * (br / 2) ** 2)))
        for t in trees:
            rmax = t["r"] * (1 + sum(a for a, _ in t["harm"]))
            sl = self._window(t["x"], t["y"], rmax + 0.1)
            dx = X[sl] - t["x"]; dy = Y[sl] - t["y"]
            rho = np.hypot(dx, dy) / self.radius_at(t, np.arctan2(dy, dx))
            g0 = self.ground_xy(t["x"], t["y"])
            dome = g0 + t["base"] + (t["top"] - t["base"]) * np.power(np.clip(1 - rho * rho, 0, 1), 0.5 * t["prof"])
            dome = dome + rng.normal(0, 0.08, dome.shape) * (rho < 1)  # leaf-scale roughness
            h[sl] = np.where(rho < 1, np.maximum(h[sl], dome), h[sl])
        self.h = h.astype(np.float32)
        self.hmax = float(h.max()); self.hmin = float(h.min())

    def _window(self, x, y, r):
        i0 = max(0, int((x - r - self.x0) / RES)); i1 = min(self.nx, int((x + r - self.x0) / RES) + 2)
        j0 = max(0, int((y - r - self.y0) / RES)); j1 = min(self.ny, int((y + r - self.y0) / RES) + 2)
        return (slice(j0, j1), slice(i0, i1))

    def ground_xy(self, x, y):
        z = self.slope[0] * x + self.slope[1] * y
        for k, (amp, ph, ang) in enumerate(self.und):
            wl = 25.0 / (k + 1)
            z = z + self.p["undulation"] * amp / 0.2 * np.sin(2 * math.pi * (x * math.cos(ang) + y * math.sin(ang)) / wl + ph)
        return z

    @staticmethod
    def radius_at(t, th):
        r = 1.0
        for k, (a, ph) in enumerate(t["harm"]):
            r = r + a * np.cos((k + 1) * th - ph)
        return t["r"] * r

    def height_at(self, x, y):
        i = np.clip(((x - self.x0) / RES + 0.5).astype(np.int64), 0, self.nx - 1)
        j = np.clip(((y - self.y0) / RES + 0.5).astype(np.int64), 0, self.ny - 1)
        out = self.h[j, i]
        outside = (x < self.x0) | (x > self.x0 + self.nx * RES) | (y < self.y0) | (y > self.y0 + self.ny * RES)
        return np.where(outside, np.float32(self.ground_xy(0.0, 0.0) - 50.0), out)  # no surface outside

    def ground_truth(self) -> List[GroundTruthTree]:
        out = []
        th = np.linspace(0, 2 * math.pi, 24, endpoint=False)
        for t in self.trees:
            r = self.radius_at(t, th)
            outline = [(float(t["x"] + rr * math.cos(a)), float(t["y"] + rr * math.sin(a))) for rr, a in zip(r, th)]
            top = float(self.ground_xy(t["x"], t["y"]) + t["top"])
            out.append(GroundTruthTree(t["id"], outline, (t["x"], t["y"]), float(np.mean(r)), round(top, 4)))
        return out


# ----------------------------------------------------------------------------- mission

def plan_mission(rng, world: World, p: dict):
    """Returns (samples, phase list, target list). samples: list of (t, x, y, z, phase, target_id, target_xy)."""
    trees = world.trees
    xs = [t["x"] for t in trees]; ys = [t["y"] for t in trees]
    y_lo, y_hi = min(ys) - 7.0, max(ys) + 7.0
    n_corr = max(1, int(math.ceil(p["rows"] / 2.0)))
    x_first = min(xs) + 0.5 * p["row_spacing"] if p["rows"] > 1 else min(xs)
    corr_x = [x_first + 2 * p["row_spacing"] * k for k in range(n_corr)]
    home = (min(xs) - 8.0, y_lo - 6.0)
    g0 = float(world.ground_xy(*home))
    pts = []  # (x, y, z, phase, burst, target, txy) at FRAME-independent fine dt
    dt = 0.02

    def line(a, b, speed, phase, target=-1, txy=(math.nan, math.nan)):
        d = math.dist(a, b)
        n = max(1, int(d / speed / dt))
        for k in range(1, n + 1):
            f = k / n
            pts.append((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), phase, False, target, txy))

    def hold(a, secs, phase, target=-1, txy=(math.nan, math.nan)):
        for _ in range(int(secs / dt)):
            pts.append((a[0], a[1], a[2], phase, False, target, txy))

    pos = (home[0], home[1], g0)
    hold(pos, 3.0, "IDLE")
    up = (home[0], home[1], g0 + SCAN_ALT)
    line(pos, up, 3.5, "IDLE"); pos = up
    start = (corr_x[0], y_lo, g0 + SCAN_ALT)
    line(pos, start, 5.0, "SCAN_APPROACH"); pos = start
    hold(pos, 1.0, "SCAN_APPROACH")
    direction = 1
    for k, cx in enumerate(corr_x):
        end = (cx, y_hi if direction > 0 else y_lo, g0 + SCAN_ALT)
        line(pos, end, p["speed"], "SCAN"); pos = end
        if k + 1 < len(corr_x):
            # U-turn: semicircle to the next corridor at cruise speed (blur + noise burst)
            nx = corr_x[k + 1]
            rad = (nx - cx) / 2.0
            ccx, ccy = cx + rad, pos[1]
            n = max(2, int(math.pi * rad / p["speed"] / dt))
            for i in range(1, n + 1):
                a = math.pi - math.pi * i / n
                yy = ccy + direction * rad * math.sin(a)
                pts.append((ccx + rad * math.cos(a), yy, pos[2], "SCAN", True, -1, (math.nan, math.nan)))
            pos = (nx, ccy, pos[2])
            direction = -direction
    hold(pos, 0.1, "PLANNING")
    # spray targets: random subset, greedy nearest order
    k = min(p["n_spray"], len(trees))
    chosen = list(rng.choice(len(trees), size=k, replace=False)) if k else []
    order = []
    cur = (pos[0], pos[1])
    while chosen:
        j = min(chosen, key=lambda i: math.hypot(trees[i]["x"] - cur[0], trees[i]["y"] - cur[1]))
        chosen.remove(j); order.append(j); cur = (trees[j]["x"], trees[j]["y"])
    for j in order:
        t = trees[j]
        txy = (t["x"] + rng.normal(0, 0.3), t["y"] + rng.normal(0, 0.3))  # navigation's idea of the centre
        gz = float(world.ground_xy(*txy))
        top_z = gz + t["top"]
        hop = (pos[0], pos[1], max(pos[2], top_z + 4.0, gz + HOP_ALT))
        line(pos, hop, 1.5, "HOPPING", t["id"], txy); pos = hop
        over = (txy[0], txy[1], pos[2])
        line(pos, over, 3.0, "HOPPING", t["id"], txy); pos = over
        down = (txy[0], txy[1], max(gz + SPRAY_ALT, top_z + 1.5))
        line(pos, down, 1.0, "HOPPING", t["id"], txy); pos = down
        hold(pos, 4.0, "SPRAYING", t["id"], txy)
    if order:
        up2 = (pos[0], pos[1], g0 + SCAN_ALT)
        line(pos, up2, 2.0, "RETURN"); pos = up2
    back = (home[0], home[1], pos[2])
    line(pos, back, 5.0, "RETURN"); pos = back
    land = (home[0], home[1], g0)
    line(pos, land, 2.0, "RETURN"); pos = land
    hold(pos, 2.0, "IDLE")
    return pts, dt


def kinematics(pts, dt):
    P = np.array([[p[0], p[1], p[2]] for p in pts])
    n = len(P)
    # smooth corners a little (1 s window) so velocities/accelerations are physical
    w = int(0.5 / dt)
    ker = np.ones(2 * w + 1) / (2 * w + 1)
    Ps = np.stack([np.convolve(np.pad(P[:, i], w, mode="edge"), ker, mode="valid") for i in range(3)], axis=1)
    V = np.gradient(Ps, dt, axis=0)
    A = np.gradient(V, dt, axis=0)
    yaw = np.zeros(n)
    last = 0.0
    for i in range(n):
        sp = math.hypot(V[i, 0], V[i, 1])
        if sp > 0.5:
            last = math.atan2(V[i, 1], V[i, 0])
        yaw[i] = last
    yaw = np.unwrap(yaw)
    yaw = np.convolve(np.pad(yaw, w, mode="edge"), ker, mode="valid")
    yaw_rate = np.gradient(yaw, dt)
    c, s = np.cos(yaw), np.sin(yaw)
    a_fwd = A[:, 0] * c + A[:, 1] * s
    a_lat = -A[:, 0] * s + A[:, 1] * c
    v_fwd = V[:, 0] * c + V[:, 1] * s
    pitch = np.arctan(a_fwd / G + 0.02 * v_fwd)
    roll = -np.arctan(a_lat / G)
    return Ps, V, yaw, pitch, roll, yaw_rate


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


# ----------------------------------------------------------------------------- sensor

class Renderer:
    def __init__(self, world: World):
        self.w = world
        cal = Calibration(stride=STRIDE, T_body_cam=list(BODY_CAM), **CAM)
        self.cal = cal
        U, V = cal.grid_uv()
        self.shape = U.shape
        self.dirs = np.stack([(U - cal.cx) / cal.fx, (V - cal.cy) / cal.fy, np.ones_like(U)], -1).reshape(-1, 3)
        self.Rbc = quat_to_rot(BODY_CAM[3:]); self.tbc = np.array(BODY_CAM[:3])

    def cam_pose(self, pos, q):
        Rwb = quat_to_rot(q)
        return Rwb @ self.Rbc, pos + Rwb @ self.tbc

    def depth(self, pos, q) -> np.ndarray:
        """Optical depth Z per grid ray (NaN = no hit), ray-cast against the heightfield."""
        R, C = self.cam_pose(np.asarray(pos), q)
        D = self.dirs @ R.T  # world direction per unit optical depth
        dz = D[:, 2]
        good = dz < -0.05
        zt, zb = self.w.hmax + 0.05, self.w.hmin - 0.05
        t0 = np.where(good, (zt - C[2]) / np.where(good, dz, -1), np.nan)
        t1 = np.where(good, (zb - C[2]) / np.where(good, dz, -1), np.nan)
        t0 = np.maximum(t0, 0.3)
        step = 0.06
        n = int(np.nanmax((t1 - t0)) / step) + 2 if np.isfinite(np.nanmax(t1 - t0)) else 2
        Z = np.full(len(D), np.nan)
        active = np.flatnonzero(good)
        prev_f = None
        tt = t0[active].copy()
        prev_t = tt.copy()
        for k in range(n):
            if not len(active):
                break
            x = C[0] + tt * D[active, 0]; y = C[1] + tt * D[active, 1]; z = C[2] + tt * D[active, 2]
            f = z - self.w.height_at(x, y)
            hit = f <= 0
            if hit.any():
                if prev_f is None:
                    Z[active[hit]] = tt[hit]
                else:
                    fp = prev_f[hit]
                    a = fp / np.maximum(fp - f[hit], 1e-9)
                    Z[active[hit]] = prev_t[hit] + a * (tt[hit] - prev_t[hit])
                keep = ~hit & (tt < t1[active])
                active, prev_f, prev_t, tt = active[keep], f[keep], tt[keep], tt[keep]
            else:
                keep = tt < t1[active]
                active, prev_f, prev_t, tt = active[keep], f[keep], tt[keep], tt[keep]
            tt = tt + step
        return Z.reshape(self.shape)


def noisy_disparity(rng, Z, p, burst: bool) -> np.ndarray:
    cal = CAM
    fB = cal["fx"] * cal["baseline"]
    d = fB / Z
    m = p["burst"] if burst else 1.0
    gh, gw = d.shape
    # foreground fattening: at depth edges a background cell takes its nearest-foreground neighbour
    pad = np.pad(np.nan_to_num(d, nan=0.0), 1, mode="edge")
    mx = np.max(np.stack([pad[1 + dy:1 + dy + gh, 1 + dx:1 + dx + gw] for dy in (-1, 0, 1) for dx in (-1, 0, 1)]), axis=0)
    edge = (mx - np.nan_to_num(d, nan=0.0)) > 0.4
    fat = edge & (rng.random(d.shape) < p["fatten"])
    d = np.where(fat, mx, d)
    # noise: white + correlated low-frequency, larger at depth edges
    sig = p["sigma_d"] * m * np.where(edge, 2.0, 1.0)
    d = d + rng.normal(0, 1, d.shape) * sig
    lo = rng.normal(0, p["corr_d"] * m, (5, 8))
    yi = np.linspace(0, 4, gh); xi = np.linspace(0, 7, gw)
    lo = np.array([np.interp(xi, np.arange(8), row) for row in lo])
    lo = np.array([np.interp(yi, np.arange(5), lo[:, c]) for c in range(gw)]).T
    d = d + lo
    # outliers (mismatches): uniform over a plausible disparity band
    med = np.nanmedian(d) if np.isfinite(d).any() else 5.0
    out = rng.random(d.shape) < p["outlier"] * m
    d = np.where(out, rng.uniform(0.5 * med, 2.0 * med, d.shape), d)
    # dropouts: iid + blobs (textureless / blurred regions)
    drop = rng.random(d.shape) < p["drop"] * m
    blob = rng.random((10, 16))
    blob = np.kron(blob, np.ones((gh // 10 + 1, gw // 16 + 1)))[:gh, :gw]
    drop |= blob < p["blob_drop"] * m
    d = np.where(drop | ~np.isfinite(d) | (d <= 0.2), np.nan, d)
    return np.round(d * 16) / 16  # SGBM-style 1/16 px quantisation


class Detector:
    def __init__(self, rng, world: World, p: dict, cal: Calibration):
        self.rng, self.w, self.p, self.cal = rng, world, p, cal
        self.missed = {}
        self.merged = {}
        th = np.linspace(0, 2 * math.pi, 16, endpoint=False)
        self.rings = {}
        for t in world.trees:
            r = World.radius_at(t, th)
            g0 = float(world.ground_xy(t["x"], t["y"]))
            xs = t["x"] + r * np.cos(th); ys = t["y"] + r * np.sin(th)
            self.rings[t["id"]] = np.r_[np.c_[xs, ys, np.full(16, g0 + t["base"])],
                                        np.c_[t["x"] + 0.6 * r * np.cos(th), t["y"] + 0.6 * r * np.sin(th),
                                              np.full(16, g0 + 0.5 * (t["base"] + t["top"]))],
                                        [[t["x"], t["y"], g0 + t["top"]]]]
        self.touch = set()
        for i, a in enumerate(world.trees):
            for b in world.trees[i + 1:]:
                if math.hypot(a["x"] - b["x"], a["y"] - b["y"]) - a["r"] - b["r"] < 0.3:
                    self.touch.add((a["id"], b["id"]))

    def project(self, R, C, P):
        pc = (P - C) @ R  # world -> optical (R = R_world_cam)
        z = pc[:, 2]
        if (z <= 0.2).any():
            return None
        return np.c_[self.cal.fx * pc[:, 0] / z + self.cal.cx, self.cal.fy * pc[:, 1] / z + self.cal.cy]

    def detect(self, R, C, burst: bool) -> np.ndarray:
        rng, p, W, H = self.rng, self.p, self.cal.width, self.cal.height
        m = p["burst"] if burst else 1.0
        vis = {}
        for tid, ring in self.rings.items():
            px = self.project(R, C, ring)
            if px is None:
                continue
            b = np.array([px[:, 0].min(), px[:, 1].min(), px[:, 0].max(), px[:, 1].max()])
            cb = np.array([max(b[0], 0), max(b[1], 0), min(b[2], W), min(b[3], H)])
            full = (b[2] - b[0]) * (b[3] - b[1])
            if cb[2] <= cb[0] or cb[3] <= cb[1] or full <= 0:
                continue
            frac = (cb[2] - cb[0]) * (cb[3] - cb[1]) / full
            if frac < 0.35:
                continue
            vis[tid] = (cb, frac)
        boxes = []
        det = {}
        for tid, (cb, frac) in vis.items():
            pm = min(0.95, p["p_miss"] * m * (1.5 if frac < 0.9 else 1.0))
            if self.missed.get(tid):
                pm = max(pm, p["miss_sticky"])
            miss = rng.random() < pm
            self.missed[tid] = miss
            if not miss:
                det[tid] = cb
        # merged boxes over touching trees (sticky per pair)
        done = set()
        for a, b in self.touch:
            if a in det and b in det and a not in done and b not in done:
                pm = p["merge"] if not self.merged.get((a, b)) else max(p["merge"], 0.7)
                mg = rng.random() < pm
                self.merged[(a, b)] = mg
                if mg:
                    A, B = det[a], det[b]
                    boxes.append((np.array([min(A[0], B[0]), min(A[1], B[1]), max(A[2], B[2]), max(A[3], B[3])]), True))
                    done |= {a, b}
        for tid, cb in det.items():
            if tid in done:
                continue
            if rng.random() < p["partial"]:
                cb = cb.copy()
                ax = rng.integers(2); side = rng.integers(2)
                mid = 0.5 * (cb[ax] + cb[ax + 2])
                cb[ax + 2 * (1 - side)] = mid  # keep one half
            boxes.append((cb, True))
        out = []
        for cb, tp in boxes:
            w, h = cb[2] - cb[0], cb[3] - cb[1]
            j = p["jitter"] * m
            cx = 0.5 * (cb[0] + cb[2]) + rng.normal(0, j * w); cy = 0.5 * (cb[1] + cb[3]) + rng.normal(0, j * h)
            w *= max(0.5, rng.normal(0.95, p["size_sd"] * m)); h *= max(0.5, rng.normal(0.95, p["size_sd"] * m))
            out.append([cx - w / 2, cy - h / 2, w, h, float(np.clip(rng.normal(0.85, 0.05), 0.3, 0.99)), 0])
        # false positives: on a visible bush, else on bare ground
        for _ in range(rng.poisson(p["fp"] * m)):
            placed = False
            if self.w.bushes and rng.random() < 0.6:
                bx, by, br, bh = self.w.bushes[rng.integers(len(self.w.bushes))]
                g = float(self.w.ground_xy(bx, by))
                px = self.project(R, C, np.array([[bx - br, by - br, g], [bx + br, by + br, g + bh], [bx - br, by + br, g], [bx + br, by - br, g]]))
                if px is not None:
                    x0, y0, x1, y1 = px[:, 0].min(), px[:, 1].min(), px[:, 0].max(), px[:, 1].max()
                    if x1 > 0 and y1 > 0 and x0 < W and y0 < H:
                        out.append([x0, y0, x1 - x0, y1 - y0, float(rng.uniform(0.4, 0.8)), 0]); placed = True
            if not placed:
                w, h = rng.uniform(50, 200), rng.uniform(50, 200)
                out.append([rng.uniform(0, W - w), rng.uniform(0, H - h), w, h, float(rng.uniform(0.4, 0.8)), 0])
        res = []
        for x, y, w, h, s, l in out:  # clip to the image like a real detector
            x0, y0, x1, y1 = max(0.0, x), max(0.0, y), min(float(W), x + w), min(float(H), y + h)
            if x1 - x0 > 8 and y1 - y0 > 8:
                res.append([round(x0), round(y0), round(x1 - x0), round(y1 - y0), s, l])
        return np.array(res, dtype=np.float32).reshape(-1, 6)


# ----------------------------------------------------------------------------- dataset

def camera_info() -> dict:
    fx, fy, cx, cy, B = CAM["fx"], CAM["fy"], CAM["cx"], CAM["cy"], CAM["baseline"]
    K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    base = {"width": CAM["width"], "height": CAM["height"], "distortion_model": "plumb_bob", "D": [0.0] * 5,
            "K": K, "R": [1, 0, 0, 0, 1, 0, 0, 0, 1]}
    return {"left": {**base, "frame_id": "synthetic_left", "P": [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]},
            "right": {**base, "frame_id": "synthetic_right", "P": [fx, 0, cx, -fx * B * 1000.0, 0, fy, cy, 0, 0, 0, 1, 0]}}


def generate(out_dir: str, level: str, seed: int, overrides: Optional[dict] = None) -> dict:
    """Write one dataset. ``overrides`` replaces level parameters (tests use small worlds)."""
    p = dict(LEVELS[level], **(overrides or {}))
    frame_hz = p.get("frame_hz", FRAME_HZ)
    rng = np.random.default_rng([seed, list(LEVELS).index(level)])
    world = World(rng, p)
    pts, dt = plan_mission(rng, world, p)
    P, V, yaw, pitch, roll, yaw_rate = kinematics(pts, dt)
    n = len(pts)
    ts = dt * np.arange(n)
    phases_s = [q[3] for q in pts]
    burst = np.array([q[4] for q in pts]) | (np.abs(yaw_rate) > 0.25)
    # bursts last 1 s beyond the turn (settling, blur)
    k1 = int(1.0 / dt)
    bb = burst.copy()
    for i in np.flatnonzero(burst):
        bb[i:i + k1] = True
    burst = bb
    os.makedirs(out_dir, exist_ok=True)
    rend = Renderer(world)
    detector = Detector(rng, world, p, rend.cal)
    # pose noise: white + random-walk bias
    bias = np.cumsum(rng.normal(0, p["pose_bias"] * math.sqrt(dt), (n, 3)), axis=0)
    t_pose_valid = next((ts[i] for i in range(n) if phases_s[i] == "SCAN_APPROACH"), 0.0) + 1.0
    meta = {"level": level, "seed": seed, "params": p, "frame_hz": frame_hz, "n_trees": len(world.trees),
            "n_missing": p["rows"] * p["per_row"] - len(world.trees), "n_bushes": len(world.bushes),
            "touching_pairs": len(detector.touch), "t_pose_valid": t_pose_valid,
            "slope": list(world.slope), "camera": CAM, "T_body_cam": BODY_CAM}
    cal = Calibration(stride=STRIDE, T_body_cam=list(BODY_CAM), frame="world",
                      note="synthetic %s seed %d" % (level, seed), **CAM)
    w = FrameWriter(os.path.join(out_dir, "frames.bin"), cal, meta)
    awake = {"SCAN_APPROACH", "SCAN", "PLANNING", "HOPPING"}
    next_frame = 0.0
    next_hb = 0.0
    cnt = {"frames": 0, "heartbeats": 0, "burst_frames": 0, "boxes": 0}
    phases, targets, pose_rows = [], [], []
    last_tgt = None
    for i in range(n):
        t = float(ts[i]); ph = phases_s[i]; tid, txy = pts[i][5], pts[i][6]
        if not phases or phases[-1][1] != ph:
            phases.append((t, ph))
            if ph not in awake:
                next_hb = t  # heartbeat at every phase change
        key = (int(tid), None if not np.isfinite(txy[0]) else (round(txy[0], 6), round(txy[1], 6)))
        if last_tgt != key:
            targets.append((t, int(tid)))
            last_tgt = key
        if not pose_rows or t - pose_rows[-1][0] >= 0.1 - 1e-9:
            pose_rows.append((t, float(P[i, 0]), float(P[i, 1]), float(P[i, 2])))
        q_true = euler_to_quat(roll[i], pitch[i], yaw[i])
        m = 2.0 if burst[i] else 1.0
        att = np.radians(p["pose_att_deg"] * m) * rng.normal(0, 1, 3) if ph in awake else np.zeros(3)
        q_meas = euler_to_quat(roll[i] + att[0], pitch[i] + att[1], yaw[i] + att[2])
        pos_meas = P[i] + bias[i] + rng.normal(0, p["pose_xy"] * m, 3)
        pose = tuple(pos_meas.tolist()) + tuple(q_meas)
        valid = t >= t_pose_valid
        if ph in awake and t + 1e-9 >= next_frame:
            next_frame = t + 1.0 / frame_hz
            Z = rend.depth(P[i], q_true)
            disp = noisy_disparity(rng, Z, p, bool(burst[i]))
            R, C = rend.cam_pose(P[i], q_true)
            boxes = detector.detect(R, C, bool(burst[i]))
            # receive time = stamp (no latency), so phase/target in the frame are those in force on receipt
            w.write(Frame(t, t, pose, valid, ph, int(tid), txy, boxes, disp.astype(np.float32)))
            cnt["frames"] += 1; cnt["boxes"] += len(boxes); cnt["burst_frames"] += int(burst[i])
        elif ph not in awake and t + 1e-9 >= next_hb:
            next_hb = t + 1.0 / HEARTBEAT_HZ
            w.write(Frame(t, t, pose, valid, ph, int(tid), txy, np.zeros((0, 6), np.float32), None))
            cnt["heartbeats"] += 1
    w.close()
    gt = world.ground_truth()
    write_ground_truth(os.path.join(out_dir, "ground_truth.json"), gt)
    write_mission(os.path.join(out_dir, "mission.json"), Mission(phases, targets, pose_rows))
    with open(os.path.join(out_dir, "camera_info.json"), "w") as f:
        json.dump(camera_info(), f, indent=1)
    meta.update(cnt)
    meta["duration_s"] = float(ts[-1])
    meta["frames_bytes"] = os.path.getsize(os.path.join(out_dir, "frames.bin"))
    with open(os.path.join(out_dir, "meta.json"), "w") as f:
        json.dump(meta, f, indent=1)
    return meta


def _job(args):
    out, level, seed = args
    m = generate(os.path.join(out, "%s_s%02d" % (level, seed)), level, seed)
    return "%s_s%02d: %d trees, %d frames, %.0f s, %.1f MB" % (level, seed, m["n_trees"], m["frames"],
                                                              m["duration_s"], m["frames_bytes"] / 1e6)


def parse_seeds(s: str) -> List[int]:
    out = []
    for part in s.split(","):
        if "-" in part:
            a, b = part.split("-")
            out += list(range(int(a), int(b) + 1))
        elif part:
            out.append(int(part))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", required=True)
    ap.add_argument("--levels", default="easy,medium,hard")
    ap.add_argument("--seeds", default="0-4")
    ap.add_argument("-j", "--jobs", type=int, default=6)
    a = ap.parse_args(argv)
    jobs = [(a.out, lv, s) for lv in a.levels.split(",") for s in parse_seeds(a.seeds)]
    os.makedirs(a.out, exist_ok=True)
    with Pool(min(a.jobs, 8, len(jobs))) as pool:
        for line in pool.imap(_job, jobs):
            print(line, flush=True)


if __name__ == "__main__":
    main()
