"""Unit tests for IK/FK and pulse conversion."""

from __future__ import annotations

import math
from pathlib import Path

from hexapod_kinematics.domain.gait_config import load_gait_config
from hexapod_kinematics.domain.kinematics import (
    JointAngles,
    LinkLengths,
    Position3D,
    ServoPulses,
    angles_to_pulses,
    forward_kinematics,
    inverse_kinematics,
    pulses_to_angles,
)
from hexapod_kinematics.domain.neutral_pose import (
    bootstrap_servo_neutral_angles,
    l_pose_neutral_angles,
    resolve_servo_neutral,
)

CAD_LENGTHS = LinkLengths(coxa=52.5, femur=42.9, tibia=54.5)
HALF_PI = math.pi / 2.0
ROOT = Path(__file__).resolve().parents[1]
GAIT = ROOT / "config" / "hexapod_gait.yml"


def test_ik_fk_round_trip() -> None:
    target = Position3D(x=90.0, y=10.0, z=-40.0)
    ok, angles = inverse_kinematics(target, CAD_LENGTHS)
    assert ok
    tip = forward_kinematics(angles, CAD_LENGTHS)
    assert abs(tip.x - target.x) < 0.5
    assert abs(tip.y - target.y) < 0.5
    assert abs(tip.z - target.z) < 0.5


def test_ik_prefers_l_branch_for_l_tip() -> None:
    tip = forward_kinematics(l_pose_neutral_angles(), CAD_LENGTHS)
    ok, angles = inverse_kinematics(
        tip, CAD_LENGTHS, prefer=l_pose_neutral_angles()
    )
    assert ok
    assert abs(angles.femur) < 1e-6
    assert abs(angles.tibia + HALF_PI) < 1e-6


def test_pulse_without_neutral_angles_is_math_delta() -> None:
    """Without servo_neutral, 1500 µs ↔ math zero (pulse I/O delta only)."""
    pulses = ServoPulses(1500, 1500, 1500)
    angles = pulses_to_angles(pulses)
    assert abs(angles.coxa) < 1e-9
    assert abs(angles.femur) < 1e-9
    assert abs(angles.tibia) < 1e-9
    back = angles_to_pulses(angles)
    assert back == pulses


def test_pulse_1500_maps_to_l_stance() -> None:
    """YAML servo_neutral is offset: 1500 µs → math (0, 0, −π/2)."""
    gait = load_gait_config(GAIT)
    neutral = resolve_servo_neutral(gait, CAD_LENGTHS)
    pulses = ServoPulses(1500, 1500, 1500)
    angles = pulses_to_angles(pulses, neutral_angles=neutral)
    assert abs(angles.coxa) < 1e-9
    assert abs(angles.femur) < 1e-9
    assert abs(angles.tibia + HALF_PI) < 1e-9
    assert abs(abs(angles.tibia) - HALF_PI) < 0.05
    back = angles_to_pulses(angles, neutral_angles=neutral)
    assert back == pulses


def test_fk_l_pose_tip_uses_tibia_effective() -> None:
    angles = l_pose_neutral_angles()
    tip = forward_kinematics(angles, CAD_LENGTHS)
    assert abs(tip.x - (CAD_LENGTHS.coxa + CAD_LENGTHS.femur)) < 1e-6
    assert abs(tip.y) < 1e-9
    assert abs(tip.z + CAD_LENGTHS.tibia) < 1e-6


def test_fk_extended_planar() -> None:
    # femur=0, tibia=0 → links extended along +x in leg plane (math zero)
    angles = JointAngles(coxa=0.0, femur=0.0, tibia=0.0)
    tip = forward_kinematics(angles, CAD_LENGTHS)
    expected = CAD_LENGTHS.coxa + CAD_LENGTHS.femur + CAD_LENGTHS.tibia
    assert abs(tip.x - expected) < 1e-6
    assert abs(tip.y) < 1e-9
    assert abs(tip.z) < 1e-9


def test_bootstrap_is_l_pose_not_ik() -> None:
    boot = bootstrap_servo_neutral_angles(CAD_LENGTHS)
    assert boot == l_pose_neutral_angles()
    gait = load_gait_config(GAIT)
    from_yaml = resolve_servo_neutral(gait, CAD_LENGTHS)
    assert abs(from_yaml.tibia - boot.tibia) < 1e-9
    assert abs(from_yaml.femur - boot.femur) < 1e-9
