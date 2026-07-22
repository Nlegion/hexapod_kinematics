"""Dynamics and sync-hexapod helpers."""

from __future__ import annotations

from pathlib import Path

from hexapod_kinematics.application.sync_hexapod import (
    patch_config_h,
    report_config_diff,
    scan_dependent_constants,
)
from hexapod_kinematics.domain.body_frame_kin import lengths_from_model
from hexapod_kinematics.domain.dynamics_leg import (
    compute_torques_for_log,
    torques_planar_femur_tibia,
)
from hexapod_kinematics.domain.kinematics import JointAngles, LinkLengths
from hexapod_kinematics.domain.masses import load_masses_config

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests" / "fixtures" / "kinematics_996_min.json"
MASSES = ROOT / "config" / "masses_996.yml"


def test_torques_finite() -> None:
    tq = torques_planar_femur_tibia(
        angles=JointAngles(0.0, 0.3, 0.5),
        omega=(0.0, 0.1, -0.1),
        alpha=(0.0, 0.0, 0.0),
        lengths=LinkLengths(52.5, 42.9, 54.5),
        masses_g={"coxa": 15, "femur": 21, "tibia": 26},
    )
    assert "M_femur_nm" in tq
    assert abs(tq["M_femur_nm"]) < 50


def test_torques_from_fake_log() -> None:
    import json

    from hexapod_kinematics.application.simulate_motion import run_simulation

    r = run_simulation(
        json_path=FIXTURE,
        gait_path=ROOT / "config" / "hexapod_gait.yml",
        mode="pulse_aligned",
        cycles=1,
        log_dir=None,
        masses_path=MASSES,
    )
    model = json.loads(FIXTURE.read_text(encoding="utf-8"))
    rows = compute_torques_for_log(
        r["pulse_aligned"]["legs"],
        lengths=lengths_from_model(model),
        masses_cfg=load_masses_config(MASSES),
        leg_ids=[0],
    )
    assert rows
    assert "M_tibia_nm" in rows[0]


def test_config_diff_and_patch(tmp_path: Path) -> None:
    import json

    model = json.loads(FIXTURE.read_text(encoding="utf-8"))
    lengths = lengths_from_model(model)
    diff = report_config_diff(lengths, model)
    assert "placeholder_mm" in diff
    cfg = tmp_path / "Config.h"
    cfg.write_text(
        "namespace Core {\nnamespace Config {\n"
        "constexpr float COXA_LENGTH = 50.0f;   // mm\n"
        "constexpr float FEMUR_LENGTH = 80.0f;  // mm\n"
        "constexpr float TIBIA_LENGTH = 120.0f; // mm\n"
        "} }\n",
        encoding="utf-8",
    )
    L = model["lengths_mm"]
    warns = patch_config_h(
        cfg,
        coxa=lengths.coxa,
        femur=lengths.femur,
        tibia=float(L["tibia"]),
        tibia_effective=float(L["tibia_effective"]),
        foot_pad_protrusion=2.0,
    )
    text = cfg.read_text(encoding="utf-8")
    assert "TIBIA_EFFECTIVE_LENGTH" in text
    assert "52.5" in text or "52.500" in text
    _ = warns
    _ = scan_dependent_constants(cfg)
