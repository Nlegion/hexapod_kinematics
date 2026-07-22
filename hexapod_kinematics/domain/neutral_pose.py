"""
Servo-neutral L-stance (math angles at 1500 µs / 90°).

L-pose: femur horizontal (0), tibia vertical down (−π/2), knee |tibia| = π/2.
Tip uses tibia_effective (print + foot pad) so body_height reaches pad contact.

``bootstrap_servo_neutral_angles`` returns the fixed L-pose — it does **not**
call IK (avoids circular dependency). Runtime offsets come from gait YAML
``servo_neutral_angles``.
"""

from __future__ import annotations

import math
from typing import Any

from hexapod_kinematics.domain.body_frame_kin import chain_to_body
from hexapod_kinematics.domain.frames_world import MountWorld
from hexapod_kinematics.domain.kinematics import (
    JointAngles,
    LinkLengths,
    forward_kinematics,
)

HALF_PI = math.pi / 2.0
DEFAULT_IK_REACH_MARGIN_MM = 5.0


def l_pose_neutral_angles() -> JointAngles:
    """Canonical L-stance math angles (independent of link lengths)."""
    return JointAngles(coxa=0.0, femur=0.0, tibia=-HALF_PI)


def bootstrap_servo_neutral_angles(
    lengths: LinkLengths | None = None,
    **_kwargs: Any,
) -> JointAngles:
    """
    Fixed L-pose (0, 0, −π/2). ``lengths`` accepted for API compatibility;
    not used (neutral is geometric, not IK-solved).
    """
    _ = lengths
    return l_pose_neutral_angles()


def servo_neutral_from_gait(gait: dict[str, Any]) -> JointAngles | None:
    """Parse ``servo_neutral_angles: [coxa, femur, tibia]`` radians, or None."""
    raw = gait.get("servo_neutral_angles")
    if raw is None:
        return None
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        raise ValueError(
            f"servo_neutral_angles must be [coxa, femur, tibia] rad, got {raw!r}"
        )
    return JointAngles(
        coxa=float(raw[0]),
        femur=float(raw[1]),
        tibia=float(raw[2]),
    )


def resolve_servo_neutral(
    gait: dict[str, Any],
    lengths: LinkLengths | None = None,
) -> JointAngles:
    """YAML angles if present, else L-pose bootstrap."""
    parsed = servo_neutral_from_gait(gait)
    if parsed is not None:
        return parsed
    return bootstrap_servo_neutral_angles(lengths)


def body_height_from_neutral(
    angles: JointAngles,
    lengths: LinkLengths,
    mount: MountWorld,
) -> float:
    """
    World-frame hip_z − foot_z for the neutral math pose.

    ``lengths.tibia`` must be tibia_effective (includes foot-pad protrusion)
    so the tip is the pad contact point.
    """
    chain = chain_to_body(angles, lengths, mount)
    return float(chain[0, 2] - chain[-1, 2])


def neutral_reach_mm(
    angles: JointAngles,
    lengths: LinkLengths,
    *,
    margin_mm: float = 0.0,
) -> float:
    """Horizontal tip reach in coxa-local, optionally pulled inward by margin."""
    tip = forward_kinematics(angles, lengths)
    reach = math.hypot(tip.x, tip.y) - float(margin_mm)
    return max(reach, lengths.coxa + 1.0)


def ik_reach_margin_from_gait(gait: dict[str, Any]) -> float:
    return float(gait.get("ik_reach_margin_mm", DEFAULT_IK_REACH_MARGIN_MM))
