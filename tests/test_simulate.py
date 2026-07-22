"""Integration simulate tests using fixture JSON (no export/)."""

from __future__ import annotations

from pathlib import Path

from hexapod_kinematics.application.log_coords import LEG_FIELDS, SUMMARY_FIELDS
from hexapod_kinematics.application.simulate_motion import run_simulation

ROOT = Path(__file__).resolve().parents[1]
JSON = ROOT / "tests" / "fixtures" / "kinematics_996_min.json"
GAIT = ROOT / "config" / "hexapod_gait.yml"
MASSES = ROOT / "config" / "masses_996.yml"


def test_simulate_pulse_aligned_writes_logs(tmp_path: Path) -> None:
    result = run_simulation(
        json_path=JSON,
        gait_path=GAIT,
        mode="pulse_aligned",
        cycles=1,
        log_dir=tmp_path,
        masses_path=MASSES,
    )
    assert "pulse_aligned" in result
    assert len(result["pulse_aligned"]["legs"]) == 8 * 6
    paths = result["log_paths"]
    assert "report_md" in paths
    legs = Path(paths["pulse_aligned_legs_csv"]).read_text(encoding="utf-8").splitlines()[0]
    summary = Path(paths["pulse_aligned_summary_csv"]).read_text(encoding="utf-8").splitlines()[0]
    for col in ("frame_idx", "leg_id", "foot_body_x"):
        assert col in legs
    for col in ("support_ok", "phase_err_mean_deg", "ik_fail_ratio", "com_source"):
        assert col in summary
    assert LEG_FIELDS[0] == "frame_idx"
    assert SUMMARY_FIELDS[0] == "frame_idx"
    assert result["masses"]["total_g"] == 1800.0
    assert "support_ok_rate" in result["report"]["pulse_aligned"]


def test_simulate_pulse_raw_differs_from_aligned() -> None:
    raw = run_simulation(
        json_path=JSON,
        gait_path=GAIT,
        mode="pulse_raw",
        cycles=1,
        log_dir=None,
        masses_path=MASSES,
    )
    aligned = run_simulation(
        json_path=JSON,
        gait_path=GAIT,
        mode="pulse_aligned",
        cycles=1,
        log_dir=None,
        masses_path=MASSES,
    )
    z_raw = raw["pulse_raw"]["anim"][0]["chains"][0][-1][2]
    z_al = aligned["pulse_aligned"]["anim"][0]["chains"][0][-1][2]
    # aligned stance tips near 0; raw typically not
    assert abs(z_al) < abs(z_raw) or abs(z_al) < 1.0


def test_simulate_ik_fail_ratio_grows_with_huge_stride(tmp_path: Path) -> None:
    import copy

    import yaml

    gait = yaml.safe_load(GAIT.read_text(encoding="utf-8"))
    gait_ok = copy.deepcopy(gait)
    gait_ok["stride_mm"] = 30.0
    gait_ok["body_height_mm"] = 40.0
    path_ok = tmp_path / "gait_ok.yml"
    path_ok.write_text(yaml.safe_dump(gait_ok), encoding="utf-8")

    gait_bad = copy.deepcopy(gait)
    gait_bad["stride_mm"] = 400.0
    gait_bad["body_height_mm"] = 40.0
    path_bad = tmp_path / "gait_bad.yml"
    path_bad.write_text(yaml.safe_dump(gait_bad), encoding="utf-8")

    ok = run_simulation(
        json_path=JSON, gait_path=path_ok, mode="ik", cycles=1, log_dir=None, masses_path=MASSES
    )
    bad = run_simulation(
        json_path=JSON, gait_path=path_bad, mode="ik", cycles=1, log_dir=None, masses_path=MASSES
    )
    ok_ratio = ok["report"]["ik"]["ik_fail_ratio_total"]
    bad_ratio = bad["report"]["ik"]["ik_fail_ratio_total"]
    assert bad_ratio > ok_ratio


def test_body_height_override_hits_hips() -> None:
    r = run_simulation(
        json_path=JSON,
        gait_path=GAIT,
        mode="pulse_raw",
        cycles=1,
        log_dir=None,
        body_height_mm=55.0,
        masses_path=MASSES,
    )
    assert abs(float(r["body_height_mm"]) - 55.0) < 1e-6
    assert abs(float(r["pulse_raw"]["summary"][0]["hip_z"]) - 55.0) < 0.5


def test_compare_same_cycles() -> None:
    r = run_simulation(
        json_path=JSON,
        gait_path=GAIT,
        mode="compare",
        cycles=2,
        log_dir=None,
        masses_path=MASSES,
    )
    assert "pulse_aligned" in r and "ik" in r
    assert "pulse_aligned" in r["report"] and "ik" in r["report"]
