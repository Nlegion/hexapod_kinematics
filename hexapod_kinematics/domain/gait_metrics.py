"""Gait analysis metrics: support polygon, phase error, strides, IK fail ratio."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from hexapod_kinematics.core.constants.world_frame import LEG_BAND
from hexapod_kinematics.domain.frames_world import MountWorld
from hexapod_kinematics.domain.masses import mass_weighted_com_xy


@dataclass(frozen=True, slots=True)
class FrameMetrics:
    com_xy: np.ndarray
    com_source: str
    n_stance: int
    support_ok: bool
    support_margin_mm: float
    duty_mean: float
    phase_err_mean_deg: float
    phase_err_max_deg: float
    ik_fail_ratio: float
    body_speed_est_mm_s: float


def geometric_com_xy(
    mounts: list[MountWorld],
    com_offset_mm: np.ndarray,
) -> np.ndarray:
    origins = np.array([m.origin for m in mounts], dtype=float)
    mean = origins.mean(axis=0)
    off = np.asarray(com_offset_mm, dtype=float).reshape(3)
    return mean[:2] + off[:2]


def point_in_triangle_2d(p: np.ndarray, tri: np.ndarray) -> bool:
    """Barycentric inclusion; tri shape (3,2)."""
    a, b, c = tri
    v0 = c - a
    v1 = b - a
    v2 = p - a
    dot00 = np.dot(v0, v0)
    dot01 = np.dot(v0, v1)
    dot02 = np.dot(v0, v2)
    dot11 = np.dot(v1, v1)
    dot12 = np.dot(v1, v2)
    denom = dot00 * dot11 - dot01 * dot01
    if abs(denom) < 1e-12:
        return False
    u = (dot11 * dot02 - dot01 * dot12) / denom
    v = (dot00 * dot12 - dot01 * dot02) / denom
    return (u >= 0) and (v >= 0) and (u + v <= 1)


def support_margin_mm(com_xy: np.ndarray, stance_xy: np.ndarray) -> float:
    """
    Signed distance from COM to nearest edge of support polygon.
    Positive = inside. For triangle use min edge distance with inside sign.
    """
    if len(stance_xy) < 3:
        return -1e9
    tri = stance_xy[:3]
    inside = point_in_triangle_2d(com_xy, tri)
    dists = []
    for i in range(3):
        a = tri[i]
        b = tri[(i + 1) % 3]
        ab = b - a
        t = np.clip(np.dot(com_xy - a, ab) / (np.dot(ab, ab) + 1e-12), 0.0, 1.0)
        proj = a + t * ab
        dists.append(float(np.linalg.norm(com_xy - proj)))
    d = min(dists) if dists else 0.0
    return d if inside else -d


def phase_errors_deg(
    *,
    group1_phases: list[float],
    group2_phases: list[float],
) -> tuple[float, float]:
    """
    Ideal alternating tripod: groups separated by 0.5 cycle (= 180°).
    phase in [0,1). Returns (mean_abs_err_deg, max_abs_err_deg).
    """
    if not group1_phases or not group2_phases:
        return 0.0, 0.0
    p1 = float(np.mean(group1_phases)) % 1.0
    p2 = float(np.mean(group2_phases)) % 1.0
    delta = abs(p2 - p1)
    delta = min(delta, 1.0 - delta)
    err_cycle = abs(delta - 0.5)
    err_deg = err_cycle * 360.0
    errs = [err_deg]
    for p in group1_phases:
        for q in group2_phases:
            d = abs((q - p) % 1.0)
            d = min(d, 1.0 - d)
            errs.append(abs(d - 0.5) * 360.0)
    return float(np.mean(errs)), float(np.max(errs))


def band_of(leg_id: int) -> str:
    return LEG_BAND.get(leg_id, "mid")


def compute_frame_metrics(
    *,
    mounts: list[MountWorld],
    foot_body_xy: dict[int, np.ndarray],
    stance_ids: list[int],
    com_offset_mm: np.ndarray,
    group1: set[int],
    phases: dict[int, float],
    ik_flags: list[bool],
    body_speed_est_mm_s: float,
    duty_instant: float,
    chains: dict[int, np.ndarray] | None = None,
    masses: dict[str, Any] | None = None,
) -> FrameMetrics:
    if masses is not None:
        com, com_source = mass_weighted_com_xy(
            mounts=mounts, chains=chains, masses=masses
        )
        # Apply gait com_offset on top of mass model offset already in masses.yml
        off = np.asarray(com_offset_mm, dtype=float).reshape(3)[:2]
        if float(np.linalg.norm(off)) > 1e-9:
            # Prefer masses.com_offset; gait offset is additive only if masses has none
            mass_off = np.asarray(
                masses.get("com_offset_mm", [0.0, 0.0, 0.0]), dtype=float
            ).reshape(3)[:2]
            if float(np.linalg.norm(mass_off)) < 1e-12:
                com = com + off
    else:
        com = geometric_com_xy(mounts, com_offset_mm)
        com_source = "geometric"

    stance_xy = np.array([foot_body_xy[i][:2] for i in stance_ids], dtype=float)
    margin = support_margin_mm(com, stance_xy) if len(stance_ids) >= 3 else -1e9
    g1p = [phases[i] for i in phases if i in group1]
    g2p = [phases[i] for i in phases if i not in group1]
    mean_err, max_err = phase_errors_deg(group1_phases=g1p, group2_phases=g2p)
    fails = sum(1 for ok in ik_flags if not ok)
    ratio = fails / max(len(ik_flags), 1)
    return FrameMetrics(
        com_xy=com,
        com_source=com_source,
        n_stance=len(stance_ids),
        support_ok=margin >= 0.0 and len(stance_ids) >= 3,
        support_margin_mm=margin,
        duty_mean=duty_instant,
        phase_err_mean_deg=mean_err,
        phase_err_max_deg=max_err,
        ik_fail_ratio=ratio,
        body_speed_est_mm_s=body_speed_est_mm_s,
    )
