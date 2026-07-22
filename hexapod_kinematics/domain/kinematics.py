"""
Hexapod leg IK / FK and pulse↔angle conversion.

Angle convention is internally consistent (IK ↔ FK round-trip).
Inspired by P:\\Arduino\\hexapod KinematicsService, but firmware FK/IK
formulas disagree with each other; this module uses the verified pair:

  femur = atan2(z, r - L_coxa) - alpha
  tibia = pi - eps
  radial = L_coxa + L_femur*cos(femur) + L_tibia*cos(femur + tibia)
  z     = L_femur*sin(femur) + L_tibia*sin(femur + tibia)
  x = radial * cos(coxa);  y = radial * sin(coxa)

Coxa-local: x/y horizontal, z up. Units: mm, radians, microseconds.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

PI = math.pi
MIN_REACH_MM = 20.0


@dataclass(frozen=True, slots=True)
class LinkLengths:
    coxa: float
    femur: float
    tibia: float  # use tibia_effective for tip contact


@dataclass(frozen=True, slots=True)
class JointAngles:
    coxa: float
    femur: float
    tibia: float


@dataclass(frozen=True, slots=True)
class ServoPulses:
    coxa: int
    femur: int
    tibia: int


@dataclass(frozen=True, slots=True)
class Position3D:
    x: float
    y: float
    z: float

    def as_array(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)


def pulse_to_rad(pulse: float, *, neutral: float = 1500.0, deg_per_us: float = 0.18) -> float:
    return math.radians((pulse - neutral) * deg_per_us)


def rad_to_pulse(rad: float, *, neutral: float = 1500.0, deg_per_us: float = 0.18) -> int:
    return int(round(neutral + math.degrees(rad) / deg_per_us))


def pulses_to_angles(
    pulses: ServoPulses,
    *,
    neutral: float = 1500.0,
    deg_per_us: float = 0.18,
    offsets: tuple[int, int, int] = (0, 0, 0),
) -> JointAngles:
    return JointAngles(
        coxa=pulse_to_rad(pulses.coxa - offsets[0], neutral=neutral, deg_per_us=deg_per_us),
        femur=pulse_to_rad(pulses.femur - offsets[1], neutral=neutral, deg_per_us=deg_per_us),
        tibia=pulse_to_rad(pulses.tibia - offsets[2], neutral=neutral, deg_per_us=deg_per_us),
    )


def angles_to_pulses(
    angles: JointAngles,
    *,
    neutral: float = 1500.0,
    deg_per_us: float = 0.18,
    offsets: tuple[int, int, int] = (0, 0, 0),
) -> ServoPulses:
    return ServoPulses(
        coxa=rad_to_pulse(angles.coxa, neutral=neutral, deg_per_us=deg_per_us) + offsets[0],
        femur=rad_to_pulse(angles.femur, neutral=neutral, deg_per_us=deg_per_us) + offsets[1],
        tibia=rad_to_pulse(angles.tibia, neutral=neutral, deg_per_us=deg_per_us) + offsets[2],
    )


def forward_kinematics(angles: JointAngles, lengths: LinkLengths) -> Position3D:
    """Angles → tip position in coxa-local frame (matches IK)."""
    # Radial-plane chain from hip: coxa along horizontal, then femur/tibia
    radial = (
        lengths.coxa
        + lengths.femur * math.cos(angles.femur)
        + lengths.tibia * math.cos(angles.femur + angles.tibia)
    )
    z = (
        lengths.femur * math.sin(angles.femur)
        + lengths.tibia * math.sin(angles.femur + angles.tibia)
    )
    return Position3D(
        x=radial * math.cos(angles.coxa),
        y=radial * math.sin(angles.coxa),
        z=z,
    )


def inverse_kinematics(
    target: Position3D,
    lengths: LinkLengths,
) -> tuple[bool, JointAngles]:
    """Tip position (coxa-local) → joint angles. Returns (ok, angles)."""
    horizontal = math.hypot(target.x, target.y)
    total = math.sqrt(target.x * target.x + target.y * target.y + target.z * target.z)
    max_reach = lengths.femur + lengths.tibia

    if total > max_reach + lengths.coxa or total < MIN_REACH_MM:
        return False, JointAngles(0.0, 0.0, 0.0)

    coxa = math.atan2(target.y, target.x)
    radial = horizontal - lengths.coxa
    leg_reach = math.hypot(radial, target.z)

    if leg_reach > max_reach - 0.1 or leg_reach < 1e-6:
        return False, JointAngles(coxa, 0.0, 0.0)

    cos_eps = (
        lengths.femur * lengths.femur
        + lengths.tibia * lengths.tibia
        - leg_reach * leg_reach
    ) / (2.0 * lengths.femur * lengths.tibia)
    if cos_eps < -1.0 or cos_eps > 1.0:
        return False, JointAngles(coxa, 0.0, 0.0)
    eps = math.acos(cos_eps)

    cos_alpha = (
        lengths.femur * lengths.femur
        + leg_reach * leg_reach
        - lengths.tibia * lengths.tibia
    ) / (2.0 * lengths.femur * leg_reach)
    if cos_alpha < -1.0 or cos_alpha > 1.0:
        return False, JointAngles(coxa, 0.0, 0.0)
    alpha = math.acos(cos_alpha)

    gamma = math.atan2(target.z, radial)
    # Consistent with FK: femur = gamma - alpha, tibia = pi - eps
    femur = gamma - alpha
    tibia = PI - eps
    return True, JointAngles(coxa=coxa, femur=femur, tibia=tibia)


def joint_chain_coxa_local(
    angles: JointAngles,
    lengths: LinkLengths,
) -> list[Position3D]:
    """Hip(0), coxa end, knee, tip in coxa-local."""
    c, f, _t = angles.coxa, angles.femur, angles.tibia
    hip = Position3D(0.0, 0.0, 0.0)
    r1 = lengths.coxa
    coxa_end = Position3D(r1 * math.cos(c), r1 * math.sin(c), 0.0)
    r2 = lengths.coxa + lengths.femur * math.cos(f)
    z2 = lengths.femur * math.sin(f)
    knee = Position3D(r2 * math.cos(c), r2 * math.sin(c), z2)
    tip = forward_kinematics(angles, lengths)
    return [hip, coxa_end, knee, tip]
