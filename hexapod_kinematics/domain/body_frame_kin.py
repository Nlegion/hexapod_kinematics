"""
Body ↔ coxa-local transforms.

Mount rotation R has columns = axis_x, axis_y, axis_z in world.
After mounts_from_model(), axis_x is horizontal **outward** from the body
(neutral coxa reach), axis_z ≈ world up (servo). Do not use raw CAD LCS X
for kinematics — it often points along the part, not the leg.

p_coxa = R^T @ (p_world - origin)
p_world = origin + R @ p_coxa

Coxa-local (firmware-style IK): x along outward reach at coxa=0, y lateral, z up.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from hexapod_kinematics.domain.frames_world import MountWorld
from hexapod_kinematics.domain.kinematics import (
    JointAngles,
    LinkLengths,
    Position3D,
    joint_chain_coxa_local,
)


@dataclass(frozen=True, slots=True)
class LegPose:
    leg_id: int
    angles: JointAngles
    foot_coxa: Position3D
    foot_body: np.ndarray  # (3,) in body/world attached frame
    chain_body: np.ndarray  # (4, 3) hip→coxa→knee→tip
    ik_ok: bool


def world_to_coxa(point_world: np.ndarray, mount: MountWorld) -> Position3D:
    local = mount.rotation.T @ (np.asarray(point_world, dtype=float) - mount.origin)
    return Position3D(float(local[0]), float(local[1]), float(local[2]))


def coxa_to_world(point_coxa: Position3D, mount: MountWorld) -> np.ndarray:
    local = np.array([point_coxa.x, point_coxa.y, point_coxa.z], dtype=float)
    return mount.origin + mount.rotation @ local


def chain_to_body(
    angles: JointAngles,
    lengths: LinkLengths,
    mount: MountWorld,
) -> np.ndarray:
    pts = joint_chain_coxa_local(angles, lengths)
    return np.vstack([coxa_to_world(p, mount) for p in pts])


def lengths_from_model(model: dict) -> LinkLengths:
    L = model["lengths_mm"]
    tibia = float(L.get("tibia_effective", L["tibia"]))
    return LinkLengths(
        coxa=float(L["coxa"]),
        femur=float(L["femur"]),
        tibia=tibia,
    )
