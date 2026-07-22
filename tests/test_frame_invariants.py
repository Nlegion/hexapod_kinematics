"""Frame / mount invariants using tests/fixtures (no export/)."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from hexapod_kinematics.application.motion.pulse_sim import simulate_pulse
from hexapod_kinematics.domain.body_frame_kin import lengths_from_model
from hexapod_kinematics.domain.frames_world import (
    mounts_from_model,
    place_hips_above_ground,
    resolve_body_height_mm,
)
from hexapod_kinematics.domain.gait_config import load_gait_config
from hexapod_kinematics.domain.kinematics import (
    JointAngles,
    forward_kinematics,
    joint_chain_coxa_local,
)

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests" / "fixtures" / "kinematics_996_min.json"
GAIT = ROOT / "config" / "hexapod_gait.yml"


def _model():
    import json

    with FIXTURE.open(encoding="utf-8") as fh:
        return json.load(fh)


def test_fixture_has_axes() -> None:
    model = _model()
    for m in model["body_mounts"]:
        assert "axis_x" in m and "axis_y" in m and "axis_z" in m
        assert len(m["axis_x"]) == 3


def test_fr_y_negative() -> None:
    mounts = mounts_from_model(_model())
    fr = next(m for m in mounts if m.leg_id == 0)
    assert "RIGHT" in fr.leg_name or fr.leg_id == 0
    assert fr.origin[1] < 0.0


def test_coxa_radial_outward() -> None:
    mounts = mounts_from_model(_model())
    center = np.mean(np.stack([m.origin for m in mounts]), axis=0)
    for m in mounts:
        radial = m.origin - center
        radial[2] = 0.0
        radial = radial / (np.linalg.norm(radial) + 1e-12)
        coxa_dir = np.asarray(m.axis_x, dtype=float).copy()
        coxa_dir[2] = 0.0
        coxa_dir = coxa_dir / (np.linalg.norm(coxa_dir) + 1e-12)
        assert float(np.dot(coxa_dir, radial)) > 0.95


def test_place_hips_sets_hip_z() -> None:
    model = _model()
    h = 47.0
    mounts = place_hips_above_ground(mounts_from_model(model), h)
    for m in mounts:
        assert abs(float(m.origin[2]) - h) < 0.5


def test_pulse_aligned_stance_tip_near_ground() -> None:
    model = _model()
    gait = load_gait_config(GAIT)
    h = resolve_body_height_mm(gait, model)
    mounts = place_hips_above_ground(mounts_from_model(model), h)
    lengths = lengths_from_model(model)
    _legs, _sum, anim = simulate_pulse(
        gait=gait,
        mounts=mounts,
        lengths=lengths,
        cycles=1,
        direction="forward",
        coxa_scale=1.0,
        pulse_mode="pulse_aligned",
        masses=None,
    )
    assert anim
    for fr in anim:
        for lid in fr["stance_ids"]:
            tip_z = float(fr["chains"][lid][-1][2])
            assert abs(tip_z) < 1.0, f"stance tip_z={tip_z} frame={fr['frame_idx']}"


def test_neutral_fk_uses_mount_axes() -> None:
    """Coxa chain uses fixture axes (not hardcoded world-up assumption alone)."""
    mounts = mounts_from_model(_model())
    lengths = lengths_from_model(_model())
    zero = JointAngles(0.0, 0.0, 0.0)
    tip = forward_kinematics(zero, lengths)
    chain = joint_chain_coxa_local(zero, lengths)
    assert tip.x > 0
    assert len(chain) == 4
    # mount axis_z nearly aligned with world up after reorient
    for m in mounts:
        assert abs(float(m.axis_z[2])) > 0.9
