"""Build stdout / markdown motion reports after simulate."""

from __future__ import annotations

from collections import defaultdict
from pathlib import Path
from typing import Any

from hexapod_kinematics.domain.masses import MassBreakdown, validate_masses


def _mean(vals: list[float]) -> float:
    return float(sum(vals) / len(vals)) if vals else 0.0


def _stride_by_band(legs: list[dict[str, Any]]) -> dict[str, float]:
    by_band: dict[str, dict[int, list[float]]] = defaultdict(lambda: defaultdict(list))
    for row in legs:
        if int(row.get("cycle", 0)) != 0:
            continue
        if row.get("role") != "support":
            continue
        band = str(row.get("band", "mid"))
        by_band[band][int(row["leg_id"])].append(float(row["foot_body_x"]))
    out: dict[str, float] = {}
    for band, legs_map in by_band.items():
        spans = []
        for xs in legs_map.values():
            if len(xs) >= 2:
                spans.append(abs(max(xs) - min(xs)))
        out[band] = float(sum(spans) / len(spans)) if spans else 0.0
    return out


def _ik_fail_stats(legs: list[dict[str, Any]]) -> dict[str, Any]:
    total = len(legs)
    fails = sum(1 for r in legs if not r.get("ik_ok", True))
    per_cycle: dict[int, list[bool]] = defaultdict(list)
    for r in legs:
        per_cycle[int(r.get("cycle", 0))].append(bool(r.get("ik_ok", True)))
    per = {
        c: (sum(1 for ok in flags if not ok) / max(len(flags), 1))
        for c, flags in sorted(per_cycle.items())
    }
    last_c = max(per) if per else 0
    return {
        "ik_fail_ratio_total": fails / max(total, 1),
        "ik_fail_ratio_per_cycle": per,
        "ik_fail_ratio_last_cycle": per.get(last_c, 0.0),
    }


def summarize_branch(
    *,
    key: str,
    legs: list[dict[str, Any]],
    summary: list[dict[str, Any]],
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    support_vals = [1.0 if r.get("support_ok") else 0.0 for r in summary]
    phase_mean = [float(r.get("phase_err_mean_deg", 0.0)) for r in summary]
    phase_max = [float(r.get("phase_err_max_deg", 0.0)) for r in summary]
    speeds = [float(r.get("body_speed_est_mm_s", 0.0)) for r in summary]
    ik_stats = _ik_fail_stats(legs)
    out: dict[str, Any] = {
        "mode": key,
        "metrics_basis": (summary[0].get("metrics_basis") if summary else key),
        "n_frames": len(summary),
        "n_leg_rows": len(legs),
        "support_ok_rate": _mean(support_vals),
        "phase_err_mean_deg": _mean(phase_mean),
        "phase_err_max_deg": max(phase_max) if phase_max else 0.0,
        "body_speed_est_mm_s_mean": _mean(speeds),
        "stride_by_band_mm": _stride_by_band(legs),
        "com_source": summary[0].get("com_source") if summary else None,
        **ik_stats,
    }
    if extra:
        out.update(extra)
    return out


def format_report_md(
    *,
    branches: dict[str, dict[str, Any]],
    masses: MassBreakdown | None = None,
    config_diff: dict[str, Any] | None = None,
    meta: dict[str, Any] | None = None,
) -> str:
    lines: list[str] = ["# Motion report", ""]
    if meta:
        lines.append("## Run")
        for k, v in meta.items():
            lines.append(f"- **{k}**: `{v}`")
        lines.append("")
    if masses is not None:
        lines.append("## Masses")
        d = masses.as_dict()
        lines.append(f"- links: {d['links_g']:.1f} g")
        lines.append(f"- servos: {d['servos_g']:.1f} g")
        lines.append(f"- body: {d['body_g']:.1f} g")
        lines.append(f"- battery_misc: {d['battery_misc_g']:.1f} g")
        lines.append(f"- **total**: {d['total_g']:.1f} g (expected {d['expected_total_g']:.1f})")
        for w in d["warnings"]:
            lines.append(f"- WARNING: {w}")
        lines.append("")
    if config_diff:
        lines.append("## Firmware length bridge")
        for k, v in config_diff.items():
            lines.append(f"- **{k}**: `{v}`")
        lines.append("")
    for name, s in branches.items():
        lines.append(f"## {name}")
        lines.append(f"- metrics_basis: `{s.get('metrics_basis')}`")
        lines.append(f"- support_ok_rate: `{s['support_ok_rate']:.4f}`")
        lines.append(f"- ik_fail_ratio_total: `{s['ik_fail_ratio_total']:.4f}`")
        lines.append(f"- ik_fail_ratio_last_cycle: `{s['ik_fail_ratio_last_cycle']:.4f}`")
        per = s.get("ik_fail_ratio_per_cycle") or {}
        if per:
            pretty = ", ".join(f"c{c}={v:.4f}" for c, v in per.items())
            lines.append(f"- ik_fail_ratio_per_cycle: {pretty}")
        lines.append(f"- phase_err_mean_deg: `{s['phase_err_mean_deg']:.3f}`")
        lines.append(f"- phase_err_max_deg: `{s['phase_err_max_deg']:.3f}`")
        lines.append(f"- body_speed_est_mm_s_mean: `{s['body_speed_est_mm_s_mean']:.3f}`")
        lines.append(f"- com_source: `{s.get('com_source')}`")
        bands = s.get("stride_by_band_mm") or {}
        if bands:
            band_s = ", ".join(f"{b}={v:.2f}" for b, v in sorted(bands.items()))
            lines.append(f"- stride_by_band_mm: {band_s}")
        lines.append("")
    if len(branches) >= 2:
        keys = list(branches.keys())
        a, b = branches[keys[0]], branches[keys[1]]
        lines.append("## Compare delta")
        lines.append(
            f"- support_ok_rate: `{a['support_ok_rate'] - b['support_ok_rate']:+.4f}`"
        )
        lines.append(
            f"- ik_fail_ratio_total: `{a['ik_fail_ratio_total'] - b['ik_fail_ratio_total']:+.4f}`"
        )
        lines.append(
            f"- phase_err_mean_deg: `{a['phase_err_mean_deg'] - b['phase_err_mean_deg']:+.3f}`"
        )
        lines.append("")
    return "\n".join(lines)


def format_report_stdout(md: str) -> str:
    """Compact printable block (same content, no heading noise)."""
    return md


def write_report(path: Path, md: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(md, encoding="utf-8")
    return path


def build_mass_breakdown(masses_cfg: dict[str, Any] | None) -> MassBreakdown | None:
    if not masses_cfg:
        return None
    return validate_masses(masses_cfg)
