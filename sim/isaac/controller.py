"""Kinematic drone controller: setpoint in, smooth rigid-body state out.

Pure Python + numpy (no Isaac imports); runs under Python 3.8 and Isaac's 3.11.

World frame is ENU (z up). Body frame is FLU (x forward, y left, z up). Velocity is
slewed towards a stopping-distance-limited desired velocity, so a static setpoint is
approached without overshoot. Roll and pitch are cosmetic: they follow
``atan(a / g)`` of the commanded horizontal acceleration, in the body frame, through a
first-order lag. Pitch is positive nose-down (rotation about +y), so forward
acceleration gives positive pitch; roll is positive left-side-up (rotation about +x),
so acceleration to the left gives negative roll.
"""
import math
from collections import namedtuple
from typing import Optional, Tuple

import numpy as np

State = namedtuple("State", ["x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw"])

G = 9.81
STOP_MARGIN = 0.9  # fraction of a_max used to plan the stop

TARGET_TAU = 0.1  # s; first-order smoothing of the tracked target, see _desired_velocity

Setpoint = Tuple[float, float, float, float, float]  # x, y, z, yaw, stamp


def wrap_angle(a: float) -> float:
    """Wrap to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class KinematicController:
    def __init__(self, v_max_xy: float = 6.0, v_max_z: float = 2.0, a_max: float = 2.0,
                 yaw_rate_max: float = math.radians(60), tilt_tau: float = 0.15,
                 setpoint_timeout: float = 1.0):
        self.v_max_xy = v_max_xy
        self.v_max_z = v_max_z
        self.a_max = a_max
        self.yaw_rate_max = yaw_rate_max
        self.tilt_tau = tilt_tau
        self.setpoint_timeout = setpoint_timeout
        self._p = np.zeros(3)
        self._v = np.zeros(3)
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._setpoint = None  # type: Optional[Setpoint]
        self._target = None  # type: Optional[np.ndarray]  # smoothed setpoint position

    @property
    def state(self) -> State:
        return State(float(self._p[0]), float(self._p[1]), float(self._p[2]),
                     float(self._v[0]), float(self._v[1]), float(self._v[2]),
                     self._roll, self._pitch, self._yaw)

    def reset(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, yaw: float = 0.0) -> None:
        """Place the drone at rest, level, and forget the last setpoint."""
        self._p = np.array([x, y, z], dtype=float)
        self._v = np.zeros(3)
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = wrap_angle(yaw)
        self._setpoint = None
        self._target = None

    def _accept_setpoint(self, setpoint: Setpoint) -> None:
        """Store a new setpoint. A malformed or non-finite one is ignored."""
        new = tuple(float(c) for c in setpoint)
        if len(new) != 5 or not all(math.isfinite(c) for c in new):
            return  # fail-safe: keep the last good setpoint
        self._setpoint = new

    def _desired_velocity(self, now: float, dt: float) -> Tuple[np.ndarray, float]:
        """Velocity the drone should have this step, and the yaw it should turn towards."""
        sp = self._setpoint
        if sp is None or now - sp[4] > self.setpoint_timeout:
            self._target = None
            return np.zeros(3), self._yaw  # hover: stop at the current position
        # A setpoint stream at 20 Hz is a staircase; tracking it raw makes the commanded
        # acceleration repeat every message. The tracked target follows the latest setpoint
        # through a short first-order lag instead. It starts on the first setpoint, then
        # approaches a static setpoint monotonically, so the no-overshoot guarantee holds.
        goal = np.array(sp[:3], dtype=float)
        if self._target is None:
            self._target = goal
        elif TARGET_TAU > 0.0:
            self._target = self._target + (goal - self._target) * (1.0 - math.exp(-dt / TARGET_TAU))
        else:
            self._target = goal
        err = self._target - self._p
        dist = float(np.linalg.norm(err))
        if dist < 1e-12:
            return np.zeros(3), sp[3]
        u = err / dist
        # The setpoint is treated as static, which guarantees no overshoot of it.
        # The acceleration budget is shared by all three axes, so the stopping-distance rule
        # runs along the 3D error direction; the group speed limits cap the speed along it.
        # Position advances by the new velocity (semi-implicit Euler), so capping the new
        # speed at dist/dt keeps one step inside the remaining distance. Braking is planned
        # for the discrete steps actually taken: a speed of n*a*dt can be brought to rest in
        # n steps of deceleration a over a distance a*dt^2*n*(n+1)/2, with STOP_MARGIN * a_max
        # so the one-step lag of the velocity tracking cannot carry the drone past the setpoint.
        a_stop = STOP_MARGIN * self.a_max
        n_steps = 0.5 * (math.sqrt(1.0 + 8.0 * dist / (a_stop * dt * dt)) - 1.0)
        speed = min(n_steps * a_stop * dt, dist / dt)
        u_xy = math.hypot(u[0], u[1])
        if u_xy > 1e-12:
            speed = min(speed, self.v_max_xy / u_xy)
        if abs(u[2]) > 1e-12:
            speed = min(speed, self.v_max_z / abs(u[2]))
        return u * speed, sp[3]

    def step(self, dt: float, now: float, setpoint: Optional[Setpoint]) -> State:
        if setpoint is not None:
            self._accept_setpoint(setpoint)
        if dt <= 0.0:
            return self.state

        v_des, yaw_target = self._desired_velocity(now, dt)
        v_old = self._v

        dv = v_des - v_old
        max_dv = self.a_max * dt
        n_dv = float(np.linalg.norm(dv))
        if n_dv > max_dv:
            n_des = float(np.linalg.norm(v_des))
            if n_des > 1e-9:
                # Acceleration budget: first cancel the velocity component across the desired
                # direction (so a turn at a corner does not drift wide), the rest goes along it.
                e = v_des / n_des
                dv_par = float(np.dot(dv, e))
                dv_perp = dv - dv_par * e
                n_perp = min(float(np.linalg.norm(dv_perp)), max_dv)
                if n_perp > 0.0:
                    dv_perp = dv_perp * (n_perp / float(np.linalg.norm(dv_perp)))
                room = math.sqrt(max(0.0, max_dv * max_dv - n_perp * n_perp))
                dv = dv_perp + max(-room, min(room, dv_par)) * e
            else:
                dv *= max_dv / n_dv
        v_new = v_old + dv
        # group limits (v_des already respects them; this guards a changed limit or rounding)
        s_xy = math.hypot(v_new[0], v_new[1])
        if s_xy > self.v_max_xy:
            v_new[0] *= self.v_max_xy / s_xy
            v_new[1] *= self.v_max_xy / s_xy
        v_new[2] = max(-self.v_max_z, min(self.v_max_z, v_new[2]))

        a = (v_new - v_old) / dt
        self._p = self._p + v_new * dt
        self._v = v_new

        # yaw: shortest path, rate limited
        dyaw = wrap_angle(yaw_target - self._yaw)
        max_dyaw = self.yaw_rate_max * dt
        self._yaw = wrap_angle(self._yaw + max(-max_dyaw, min(max_dyaw, dyaw)))

        # tilt: horizontal acceleration into the body frame, atan(a/g), first-order lag
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        a_fwd = a[0] * c + a[1] * s
        a_left = -a[0] * s + a[1] * c
        pitch_target = math.atan(a_fwd / G)
        roll_target = -math.atan(a_left / G)
        k = 1.0 - math.exp(-dt / self.tilt_tau) if self.tilt_tau > 0.0 else 1.0
        self._pitch += (pitch_target - self._pitch) * k
        self._roll += (roll_target - self._roll) * k
        return self.state
