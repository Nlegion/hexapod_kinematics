"""Unit tests for IK/FK and pulse conversion."""

from __future__ import annotations

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

CAD_LENGTHS = LinkLengths(coxa=52.5, femur=42.9, tibia=54.5)


def test_ik_fk_round_trip() -> None:
    target = Position3D(x=90.0, y=10.0, z=-40.0)
    ok, angles = inverse_kinematics(target, CAD_LENGTHS)
    assert ok
    tip = forward_kinematics(angles, CAD_LENGTHS)
    assert abs(tip.x - target.x) < 0.5
    assert abs(tip.y - target.y) < 0.5
    assert abs(tip.z - target.z) < 0.5


def test_pulse_neutral_zero_angles() -> None:
    pulses = ServoPulses(1500, 1500, 1500)
    angles = pulses_to_angles(pulses)
    assert abs(angles.coxa) < 1e-9
    assert abs(angles.femur) < 1e-9
    assert abs(angles.tibia) < 1e-9
    back = angles_to_pulses(angles)
    assert back == pulses


def test_fk_extended_planar() -> None:
    # femur=0, tibia=0 → links extended along +x in leg plane
    angles = JointAngles(coxa=0.0, femur=0.0, tibia=0.0)
    tip = forward_kinematics(angles, CAD_LENGTHS)
    expected = CAD_LENGTHS.coxa + CAD_LENGTHS.femur + CAD_LENGTHS.tibia
    assert abs(tip.x - expected) < 1e-6
    assert abs(tip.y) < 1e-9
    assert abs(tip.z) < 1e-9
