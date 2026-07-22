"""Pulse gait and IK gait tests."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from hexapod_kinematics.domain.frames_world import MountWorld
from hexapod_kinematics.domain.gait_ik import foot_target_body, leg_phase, swing_height
from hexapod_kinematics.domain.gait_metrics import phase_errors_deg
from hexapod_kinematics.domain.gait_pulse import iter_pulse_gait

ROOT = Path(__file__).resolve().parents[1]
GAIT_PATH = ROOT / "config" / "hexapod_gait.yml"


def _gait() -> dict:
    with GAIT_PATH.open(encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def test_pulse_cycle_eight_steps_and_groups() -> None:
    gait = _gait()
    frames = iter_pulse_gait(gait, cycles=1, direction="forward")
    # 2 phases × 4 steps
    assert len(frames) == 8
    g1 = set(gait["tripod_group_1"])
    g2 = set(gait["tripod_group_2"])
    assert g1 == {0, 4, 2}
    assert g2 == {5, 1, 3}
    phase1 = frames[0]
    for lid in g1:
        assert phase1.roles[lid] == "transfer"
    for lid in g2:
        assert phase1.roles[lid] == "support"
    phase2 = frames[4]
    for lid in g2:
        assert phase2.roles[lid] == "transfer"


def test_phase_err_ideal_tripod() -> None:
    mean_e, max_e = phase_errors_deg(
        group1_phases=[0.0, 0.0, 0.0],
        group2_phases=[0.5, 0.5, 0.5],
    )
    assert mean_e < 1e-6
    assert max_e < 1e-6


def test_stance_foot_world_fixed() -> None:
    """Body advances; stance foot world position constant within 0.1 mm."""
    gait = _gait()
    duty = float(gait["stance_duty"])
    stride = float(gait["stride_mm"])
    mount = MountWorld(
        leg_id=0,
        leg_name="FR",
        origin=np.array([100.0, -80.0, 0.0]),
        axis_x=np.array([1.0, 0.0, 0.0]),
        axis_y=np.array([0.0, 1.0, 0.0]),
        axis_z=np.array([0.0, 0.0, 1.0]),
        yaw_deg=-40.0,
    )
    neutral = np.array([140.0, -80.0])
    group1 = set(gait["tripod_group_1"])

    worlds = []
    for s in (0.05, 0.15, 0.25, 0.35):
        s_leg, role = leg_phase(s, 0, group1, duty)
        assert role == "support"
        body = foot_target_body(
            s_leg=s_leg,
            role=role,
            duty=duty,
            mount=mount,
            stride_mm=stride,
            step_height_mm=20.0,
            body_height_mm=45.0,
            neutral_xy=neutral,
            profile="poly5",
        )
        # Constant body speed: stride per full cycle
        body_origin_x = stride * s
        foot_world_x = body[0] + body_origin_x
        worlds.append(foot_world_x)

    assert max(worlds) - min(worlds) < 0.1


def test_swing_profiles_peak() -> None:
    for profile in ("ellipse", "cycloid", "poly5"):
        h_mid = swing_height(0.5, 20.0, profile)  # type: ignore[arg-type]
        h0 = swing_height(0.0, 20.0, profile)  # type: ignore[arg-type]
        h1 = swing_height(1.0, 20.0, profile)  # type: ignore[arg-type]
        assert h0 < 1e-6
        assert h1 < 1e-6
        assert h_mid > 5.0
