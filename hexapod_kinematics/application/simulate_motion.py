"""Orchestrate pulse / IK / compare motion simulation and logging."""

from __future__ import annotations

import copy
import json
import logging
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Literal

from hexapod_kinematics.application.log_coords import (
    make_log_paths,
    write_legs_csv,
    write_legs_jsonl,
    write_summary_csv,
)
from hexapod_kinematics.application.motion import (
    measure_pulse_stride_mm,
    simulate_ik,
    simulate_pulse,
)
from hexapod_kinematics.application.motion_report import (
    build_mass_breakdown,
    format_report_md,
    summarize_branch,
    write_report,
)
from hexapod_kinematics.application.sync_hexapod import report_config_diff
from hexapod_kinematics.domain.body_frame_kin import lengths_from_model
from hexapod_kinematics.domain.frames_world import (
    mounts_from_model,
    place_hips_above_ground,
    resolve_body_height_mm,
)
from hexapod_kinematics.domain.gait_config import (
    assert_matches_firmware_snapshot,
    load_gait_config,
)
from hexapod_kinematics.domain.gait_pulse import Direction, estimate_coxa_scale_for_stride
from hexapod_kinematics.domain.masses import load_masses_config

logger = logging.getLogger(__name__)

Mode = Literal[
    "pulse",
    "pulse_raw",
    "pulse_aligned",
    "ik",
    "compare",
]

PULSE_ALIAS = {
    "pulse": "pulse_aligned",
    "pulse_raw": "pulse_raw",
    "pulse_aligned": "pulse_aligned",
}


def load_kinematics_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as fh:
        return json.load(fh)


def _default_masses_path() -> Path:
    return Path(__file__).resolve().parents[2] / "config" / "masses_996.yml"


def run_simulation(
    *,
    json_path: Path,
    gait_path: Path,
    mode: Mode = "pulse",
    cycles: int = 2,
    direction: Direction = "forward",
    log_dir: Path | None = None,
    scale_pulse_to_stride: float | None = None,
    body_height_mm: float | None = None,
    masses_path: Path | None = None,
    write_report_file: bool = True,
) -> dict[str, Any]:
    model = load_kinematics_json(json_path)
    gait = load_gait_config(gait_path)
    for warn in assert_matches_firmware_snapshot(gait):
        logger.warning("%s", warn)

    masses_file = masses_path or _default_masses_path()
    masses_cfg = None
    mass_breakdown = None
    if masses_file.is_file():
        masses_cfg = load_masses_config(masses_file)
        mass_breakdown = build_mass_breakdown(masses_cfg)
        for w in mass_breakdown.warnings:
            logger.warning("masses: %s", w)
    else:
        logger.warning("masses config missing: %s", masses_file)

    if body_height_mm is not None:
        gait = copy.deepcopy(gait)
        gait["body_height_mm"] = float(body_height_mm)

    mounts = mounts_from_model(model)
    lengths = lengths_from_model(model)
    body_height = resolve_body_height_mm(gait, model)
    mounts = place_hips_above_ground(mounts, body_height)

    resolved_mode = mode
    pulse_mode = PULSE_ALIAS.get(mode)
    logger.info(
        "simulate mode=%s pulse=%s lengths=%.1f/%.1f/%.1f body_height=%.1f",
        resolved_mode,
        pulse_mode,
        lengths.coxa,
        lengths.femur,
        lengths.tibia,
        body_height,
    )

    results: dict[str, Any] = {
        "mode": resolved_mode,
        "body_height_mm": body_height,
        "lengths": {
            "coxa": lengths.coxa,
            "femur": lengths.femur,
            "tibia": lengths.tibia,
        },
        "masses": mass_breakdown.as_dict() if mass_breakdown else None,
    }
    config_diff = report_config_diff(lengths, model)
    results["config_diff"] = config_diff

    coxa_scale = 1.0
    branches_for_report: dict[str, dict[str, Any]] = {}

    if mode in ("pulse", "pulse_raw", "pulse_aligned", "compare"):
        pm = pulse_mode or "pulse_aligned"
        if mode == "compare":
            pm = "pulse_aligned"
        leg_p, sum_p, anim_p = simulate_pulse(
            gait=gait,
            mounts=mounts,
            lengths=lengths,
            cycles=max(cycles, 1),
            direction=direction,
            coxa_scale=1.0,
            pulse_mode=pm,  # type: ignore[arg-type]
            masses=masses_cfg,
        )
        measured = measure_pulse_stride_mm(leg_p)
        results["pulse_stride_measured_mm"] = measured
        if scale_pulse_to_stride is not None:
            coxa_scale = estimate_coxa_scale_for_stride(
                current_stride_mm=measured or 1.0,
                target_stride_mm=float(scale_pulse_to_stride),
            )
            leg_p, sum_p, anim_p = simulate_pulse(
                gait=gait,
                mounts=mounts,
                lengths=lengths,
                cycles=cycles,
                direction=direction,
                coxa_scale=coxa_scale,
                pulse_mode=pm,  # type: ignore[arg-type]
                masses=masses_cfg,
            )
            results["pulse_stride_measured_mm"] = measure_pulse_stride_mm(leg_p)
            results["coxa_scale"] = coxa_scale
        key = pm
        results[key] = {"legs": leg_p, "summary": sum_p, "anim": anim_p}
        if mode == "compare":
            results["pulse_aligned"] = results[key]
        branches_for_report[key] = summarize_branch(
            key=key,
            legs=leg_p,
            summary=sum_p,
            extra={"pulse_stride_measured_mm": results["pulse_stride_measured_mm"]},
        )

    if mode in ("ik", "compare"):
        gait_ik = copy.deepcopy(gait)
        if mode == "compare":
            measured = float(
                results.get("pulse_stride_measured_mm") or gait["stride_mm"]
            )
            gait_ik["stride_mm"] = measured
            logger.info("compare: IK stride_mm set to pulse measured %.2f", measured)
        leg_i, sum_i, anim_i = simulate_ik(
            gait=gait_ik,
            mounts=mounts,
            lengths=lengths,
            cycles=cycles,
            direction=direction,
            body_height_mm=body_height,
            masses=masses_cfg,
        )
        results["ik"] = {"legs": leg_i, "summary": sum_i, "anim": anim_i}
        branches_for_report["ik"] = summarize_branch(
            key="ik",
            legs=leg_i,
            summary=sum_i,
            extra={"stride_mm": float(gait_ik.get("stride_mm", 0.0))},
        )

    # convenience alias
    if mode in ("pulse", "pulse_aligned") and "pulse_aligned" in results:
        results["pulse"] = results["pulse_aligned"]
    elif mode == "pulse_raw" and "pulse_raw" in results:
        results["pulse"] = results["pulse_raw"]

    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    report_md = format_report_md(
        branches=branches_for_report,
        masses=mass_breakdown,
        config_diff=config_diff,
        meta={
            "mode": resolved_mode,
            "cycles": cycles,
            "direction": direction,
            "body_height_mm": body_height,
            "json": str(json_path),
            "gait": str(gait_path),
        },
    )
    results["report_md"] = report_md
    results["report"] = branches_for_report

    if log_dir is not None:
        paths: dict[str, str] = {}
        log_keys: list[str] = []
        if mode == "ik":
            log_keys = ["ik"]
        elif mode == "compare":
            log_keys = ["pulse_aligned", "ik"]
        elif mode in ("pulse", "pulse_aligned"):
            log_keys = ["pulse_aligned"]
        else:
            log_keys = ["pulse_raw"]

        for key in log_keys:
            if key not in results:
                continue
            lp, jp, sp = make_log_paths(log_dir, key)
            write_legs_csv(lp, results[key]["legs"])
            write_legs_jsonl(jp, results[key]["legs"], meta={"mode": key})
            write_summary_csv(sp, results[key]["summary"])
            paths[f"{key}_legs_csv"] = str(lp)
            paths[f"{key}_summary_csv"] = str(sp)

        if write_report_file:
            report_path = Path(log_dir) / f"motion_{resolved_mode}_{stamp}_report.md"
            write_report(report_path, report_md)
            paths["report_md"] = str(report_path)
        results["log_paths"] = paths

    return results
