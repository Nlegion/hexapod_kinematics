"""
IK-based alternating tripod foot trajectories.

Stance: foot fixed in world; body advances → foot retreats in body frame.
Swing: configurable height profile (ellipse | cycloid | poly5).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Literal

import numpy as np

from hexapod_kinematics.domain.body_frame_kin import world_to_coxa
from hexapod_kinematics.domain.frames_world import MountWorld
from hexapod_kinematics.domain.kinematics import (
    JointAngles,
    LinkLengths,
    Position3D,
    inverse_kinematics,
)
from hexapod_kinematics.domain.neutral_pose import l_pose_neutral_angles

SwingProfile = Literal["ellipse", "cycloid", "poly5"]


def poly5(u: float) -> float:
    """Minimum-jerk rest-to-rest on [0,1] → [0,1]."""
    u = min(1.0, max(0.0, u))
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def swing_height(u: float, step_height: float, profile: SwingProfile) -> float:
    u = min(1.0, max(0.0, u))
    if profile == "ellipse":
        return step_height * math.sin(math.pi * u)
    if profile == "cycloid":
        # half-cycloid vertical: (1 - cos(2πu))/2 scaled — peak at mid
        return step_height * 0.5 * (1.0 - math.cos(2.0 * math.pi * u))
    # poly5 up then down via sin envelope on poly progress
    s = poly5(u)
    return step_height * math.sin(math.pi * s)


def swing_progress(u: float, profile: SwingProfile) -> float:
    """Horizontal progress 0→1 along swing."""
    if profile == "poly5":
        return poly5(u)
    if profile == "cycloid":
        return (u - math.sin(2.0 * math.pi * u) / (2.0 * math.pi)) if u > 0 else 0.0
    return u


@dataclass(frozen=True, slots=True)
class IkLegSample:
    leg_id: int
    phase_s: float
    role: str
    target_body: np.ndarray
    target_coxa: Position3D
    angles: JointAngles
    ik_ok: bool


def leg_phase(s_global: float, leg_id: int, group1: set[int], duty: float) -> tuple[float, str]:
    """Return (phase in [0,1), role). Group2 is offset by 0.5."""
    offset = 0.0 if leg_id in group1 else 0.5
    s = (s_global + offset) % 1.0
    role = "support" if s < duty else "transfer"
    return s, role


def foot_target_body(
    *,
    s_leg: float,
    role: str,
    duty: float,
    mount: MountWorld,
    stride_mm: float,
    step_height_mm: float,
    body_height_mm: float,
    neutral_xy: np.ndarray,
    profile: SwingProfile,
) -> np.ndarray:
    """
    Foot position in body frame (world-aligned axes attached to body).

    Neutral stance XY at hip + radial offset.
    Ground plane is z=0; hips sit at body_height (see place_hips_above_ground).
    """
    ground_z = 0.0
    # Body travels `stride_mm` per cycle; stance foot retreats by stride*duty
    # so that foot_world = body_x(t) + foot_body stays fixed (constant body speed).
    stroke = stride_mm * duty
    if role == "support":
        u = s_leg / duty if duty > 1e-9 else 0.0
        x_off = stroke * (0.5 - u)
        return np.array(
            [neutral_xy[0] + x_off, neutral_xy[1], ground_z],
            dtype=float,
        )

    u = (s_leg - duty) / (1.0 - duty) if duty < 1.0 - 1e-9 else 1.0
    prog = swing_progress(u, profile)
    x_off = stroke * (-0.5 + prog)
    z = ground_z + swing_height(u, step_height_mm, profile)
    return np.array([neutral_xy[0] + x_off, neutral_xy[1], z], dtype=float)


def default_neutral_xy(
    mount: MountWorld,
    lengths: LinkLengths,
    gait: dict[str, Any],
) -> np.ndarray:
    """Hip XY + outward horizontal unit * neutral_reach (with IK margin)."""
    reach = gait.get("neutral_reach_mm")
    if reach is None:
        from hexapod_kinematics.domain.neutral_pose import (
            ik_reach_margin_from_gait,
            neutral_reach_mm,
            resolve_servo_neutral,
        )

        angles = resolve_servo_neutral(gait, lengths)
        reach = neutral_reach_mm(
            angles,
            lengths,
            margin_mm=ik_reach_margin_from_gait(gait),
        )
    reach = float(reach)
    radial = mount.origin.copy()
    radial[2] = 0.0
    n = np.linalg.norm(radial)
    if n < 1e-9:
        radial = np.array([1.0, 0.0, 0.0])
    else:
        radial = radial / n
    return mount.origin[:2] + radial[:2] * reach


def sample_ik_frame(
    *,
    s_global: float,
    mounts: list[MountWorld],
    lengths: LinkLengths,
    gait: dict[str, Any],
    body_height_mm: float,
    last_angles: dict[int, JointAngles] | None = None,
) -> list[IkLegSample]:
    group1 = set(int(x) for x in gait["tripod_group_1"])
    duty = float(gait.get("stance_duty", 0.5))
    stride = float(gait.get("stride_mm", 40.0))
    step_h = float(gait.get("step_height_mm", 20.0))
    profile = str(gait.get("swing_profile", "poly5"))
    if profile not in ("ellipse", "cycloid", "poly5"):
        profile = "poly5"

    out: list[IkLegSample] = []
    for mount in mounts:
        s_leg, role = leg_phase(s_global, mount.leg_id, group1, duty)
        neutral = default_neutral_xy(mount, lengths, gait)
        target_body = foot_target_body(
            s_leg=s_leg,
            role=role,
            duty=duty,
            mount=mount,
            stride_mm=stride,
            step_height_mm=step_h,
            body_height_mm=body_height_mm,
            neutral_xy=neutral,
            profile=profile,  # type: ignore[arg-type]
        )
        target_coxa = world_to_coxa(target_body, mount)
        prefer = (
            last_angles[mount.leg_id]
            if last_angles and mount.leg_id in last_angles
            else l_pose_neutral_angles()
        )
        ok, angles = inverse_kinematics(target_coxa, lengths, prefer=prefer)
        if not ok and last_angles and mount.leg_id in last_angles:
            angles = last_angles[mount.leg_id]
        out.append(
            IkLegSample(
                leg_id=mount.leg_id,
                phase_s=s_leg,
                role=role,
                target_body=target_body,
                target_coxa=target_coxa,
                angles=angles,
                ik_ok=ok,
            )
        )
    return out
