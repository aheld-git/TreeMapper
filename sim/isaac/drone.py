"""Isaac-only: the kinematic drone (``/World/drone``) and its TF.

``Drone.step`` advances the ``KinematicController`` by one physics step and writes the resulting pose
to the USD Xform (body frame FLU = the prim's own axes). ``tf_message`` builds the ``world -> body``
transform for a ROS 2 ``tf2_msgs/TFMessage``. The cameras are children of the drone prim
(see ``cameras.py``), so they follow roll, pitch and yaw.
"""
from typing import Optional, Sequence

import numpy as np
from pxr import Gf, Usd, UsdGeom

from isaac import frames
from isaac.controller import KinematicController, Setpoint, State

DRONE_PATH = "/World/drone"
WORLD_FRAME = "world"
BODY_FRAME = "body"


def _pose_matrix(state: State) -> Gf.Matrix4d:
    R = frames.body_rotation(state.roll, state.pitch, state.yaw)
    # USD matrices are row-vector: rows 0..2 are the body axes in the world frame, row 3 the position.
    return Gf.Matrix4d(
        float(R[0, 0]), float(R[1, 0]), float(R[2, 0]), 0.0,
        float(R[0, 1]), float(R[1, 1]), float(R[2, 1]), 0.0,
        float(R[0, 2]), float(R[1, 2]), float(R[2, 2]), 0.0,
        float(state.x), float(state.y), float(state.z), 1.0)


class Drone:
    def __init__(self, stage: Usd.Stage, controller: KinematicController,
                 start_xyz: Sequence[float], start_yaw: float = 0.0, path: str = DRONE_PATH):
        self.controller = controller
        self.path = path
        xform = UsdGeom.Xform.Define(stage, path)
        self._op = xform.AddTransformOp()
        self.controller.reset(float(start_xyz[0]), float(start_xyz[1]), float(start_xyz[2]), float(start_yaw))
        self.state = self.controller.state
        self._op.Set(_pose_matrix(self.state))

    def step(self, dt: float, now: float, setpoint: Optional[Setpoint]) -> State:
        """Advance the controller by ``dt`` to sim time ``now`` and move the prim."""
        self.state = self.controller.step(dt, now, setpoint)
        self._op.Set(_pose_matrix(self.state))
        return self.state

    def rotation(self) -> np.ndarray:
        return frames.body_rotation(self.state.roll, self.state.pitch, self.state.yaw)


def world_to_body_transform(state: State, stamp_sec: int, stamp_nanosec: int):
    """geometry_msgs/TransformStamped (world -> body) for ``state`` (import deferred: needs rclpy's messages)."""
    from geometry_msgs.msg import TransformStamped
    t = TransformStamped()
    t.header.stamp.sec = int(stamp_sec)
    t.header.stamp.nanosec = int(stamp_nanosec)
    t.header.frame_id = WORLD_FRAME
    t.child_frame_id = BODY_FRAME
    t.transform.translation.x = state.x
    t.transform.translation.y = state.y
    t.transform.translation.z = state.z
    qx, qy, qz, qw = frames.rot_to_quat(frames.body_rotation(state.roll, state.pitch, state.yaw))
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
    return t
