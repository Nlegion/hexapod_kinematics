"""CSV / JSONL writers for motion legs and summary logs."""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

LEG_FIELDS = [
    "frame_idx",
    "t_ms",
    "mode",
    "metrics_basis",
    "cycle",
    "phase",
    "step",
    "direction",
    "leg_id",
    "leg_name",
    "band",
    "role",
    "pulse_c",
    "pulse_f",
    "pulse_t",
    "angle_c_rad",
    "angle_f_rad",
    "angle_t_rad",
    "foot_body_x",
    "foot_body_y",
    "foot_body_z",
    "foot_raw_x",
    "foot_raw_y",
    "foot_raw_z",
    "foot_coxa_x",
    "foot_coxa_y",
    "foot_coxa_z",
    "align_dz_mm",
    "ik_ok",
    "reach_mm",
    "stride_mm",
]

SUMMARY_FIELDS = [
    "frame_idx",
    "t_ms",
    "mode",
    "metrics_basis",
    "cycle",
    "phase",
    "step",
    "com_x",
    "com_y",
    "com_source",
    "n_stance",
    "support_ok",
    "support_margin_mm",
    "duty_mean",
    "phase_err_mean_deg",
    "phase_err_max_deg",
    "ik_fail_ratio",
    "body_speed_est_mm_s",
    "align_dz_mm",
    "hip_z",
]


def _stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def make_log_paths(log_dir: Path, mode: str) -> tuple[Path, Path, Path]:
    log_dir.mkdir(parents=True, exist_ok=True)
    base = f"motion_{mode}_{_stamp()}"
    return (
        log_dir / f"{base}_legs.csv",
        log_dir / f"{base}_legs.jsonl",
        log_dir / f"{base}_summary.csv",
    )


def write_legs_csv(path: Path, rows: Iterable[dict[str, Any]]) -> Path:
    rows = list(rows)
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=LEG_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return path


def write_legs_jsonl(path: Path, rows: Iterable[dict[str, Any]], meta: dict | None = None) -> Path:
    with path.open("w", encoding="utf-8") as fh:
        if meta:
            fh.write(json.dumps({"_meta": meta}, ensure_ascii=False) + "\n")
        for row in rows:
            fh.write(json.dumps(row, ensure_ascii=False) + "\n")
    return path


def write_summary_csv(path: Path, rows: Iterable[dict[str, Any]]) -> Path:
    rows = list(rows)
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=SUMMARY_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return path


def to_plain(obj: Any) -> Any:
    if is_dataclass(obj):
        return asdict(obj)
    if isinstance(obj, dict):
        return {k: to_plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [to_plain(x) for x in obj]
    return obj
