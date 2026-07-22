"""L-stance / preview-geometry consistency (pulse @ 1500 µs)."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from hexapod_kinematics.domain.body_frame_kin import chain_to_body, lengths_from_model
from hexapod_kinematics.domain.frames_world import (
    mounts_from_model,
    place_hips_above_ground,
    resolve_body_height_mm,
)
from hexapod_kinematics.domain.gait_config import load_gait_config
from hexapod_kinematics.domain.kinematics import (
    ServoPulses,
    forward_kinematics,
    pulses_to_angles,
)
from hexapod_kinematics.domain.neutral_pose import (
    l_pose_neutral_angles,
    resolve_servo_neutral,
)

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests" / "fixtures" / "kinematics_996_min.json"
GAIT = ROOT / "config" / "hexapod_gait.yml"
HALF_PI = math.pi / 2.0


def _model() -> dict:
    with FIXTURE.open(encoding="utf-8") as fh:
        return json.load(fh)


def test_body_height_equals_tibia_effective() -> None:
    model = _model()
    gait = load_gait_config(GAIT)
    lengths = lengths_from_model(model)
    h = resolve_body_height_mm(gait, model)
    assert abs(h - lengths.tibia) < 1.0


def test_place_hips_l_pose_tips_on_ground() -> None:
    model = _model()
    gait = load_gait_config(GAIT)
    lengths = lengths_from_model(model)
    h = resolve_body_height_mm(gait, model)
    mounts = place_hips_above_ground(mounts_from_model(model), h)
    neutral = resolve_servo_neutral(gait, lengths)
    for mount in mounts:
        chain = chain_to_body(neutral, lengths, mount)
        assert abs(float(chain[-1, 2])) < 1.0


def test_pulse_1500_world_hierarchy_and_tibia_vertical() -> None:
    """
    World frame @1500: knee_z ≈ hip_z, tip_z ≈ hip_z − tibia_eff;
    tibia axis aligned with world −up (angle to vertical ≈ 0°).
    """
    model = _model()
    gait = load_gait_config(GAIT)
    lengths = lengths_from_model(model)
    h = resolve_body_height_mm(gait, model)
    mounts = place_hips_above_ground(mounts_from_model(model), h)
    neutral = resolve_servo_neutral(gait, lengths)
    angles = pulses_to_angles(
        ServoPulses(1500, 1500, 1500),
        neutral_angles=neutral,
    )
    assert abs(abs(angles.tibia) - HALF_PI) < 0.05
    assert abs(angles.femur) < 0.05

    world_up = np.array([0.0, 0.0, 1.0])
    for mount in mounts:
        chain = chain_to_body(angles, lengths, mount)
        hip = chain[0]
        knee = chain[2]
        tip = chain[3]
        assert abs(float(knee[2] - hip[2])) < 1.0
        assert abs(float(tip[2] - (hip[2] - lengths.tibia))) < 1.0
        tibia_axis = tip - knee
        tn = np.linalg.norm(tibia_axis)
        assert tn > 1e-6
        tibia_dir = tibia_axis / tn
        # Vertical down: angle between tibia and -world_up ≈ 0
        cos_to_down = float(np.dot(tibia_dir, -world_up))
        angle_to_vertical = math.degrees(math.acos(min(1.0, max(-1.0, cos_to_down))))
        assert angle_to_vertical < 5.0, f"leg {mount.leg_id} angle={angle_to_vertical}"


def test_l_pose_fk_tip_point() -> None:
    lengths = lengths_from_model(_model())
    tip = forward_kinematics(l_pose_neutral_angles(), lengths)
    assert abs(tip.x - (lengths.coxa + lengths.femur)) < 1e-6
    assert abs(tip.z + lengths.tibia) < 1e-6
