"""Body-frame yaw and mount auto-assignment helpers."""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass

from hexapod_kinematics.domain.frames import Frame3D, Vec3

logger = logging.getLogger(__name__)


@dataclass(frozen=True, slots=True)
class BodyBasis:
    forward: Vec3
    up: Vec3
    right: Vec3


def build_body_basis(forward: Vec3, up: Vec3) -> BodyBasis:
    f = forward.normalized()
    u = up.normalized()
    right = f.cross(u).normalized()
    # Re-orthogonalize up in case inputs were not perpendicular
    u = right.cross(f).normalized()
    return BodyBasis(forward=f, up=u, right=right)


def horizontal_components(
    point: Vec3,
    center: Vec3,
    basis: BodyBasis,
) -> tuple[float, float]:
    delta = point - center
    forward_comp = delta.dot(basis.forward)
    right_comp = delta.dot(basis.right)
    return forward_comp, right_comp


def horizontal_radius(point: Vec3, center: Vec3, basis: BodyBasis) -> float:
    f, r = horizontal_components(point, center, basis)
    return math.hypot(f, r)


def yaw_angle_rad(point: Vec3, center: Vec3, basis: BodyBasis) -> float:
    """Angle in horizontal plane: 0 along forward, positive toward right."""
    f, r = horizontal_components(point, center, basis)
    return math.atan2(r, f)


def convert_yaw(angle_rad: float, unit: str) -> float:
    if unit == "rad":
        return angle_rad
    if unit == "deg":
        return math.degrees(angle_rad)
    raise ValueError(f"unsupported yaw_unit={unit!r}")


def coxa_axis_level_ok(
    axis_z: Vec3,
    up: Vec3,
    tol: float,
) -> bool:
    """True when servo Z is horizontal (perpendicular to body up)."""
    return abs(axis_z.normalized().dot(up.normalized())) <= tol


def coxa_axis_vertical_ok(
    axis_z: Vec3,
    up: Vec3,
    tol: float,
) -> bool:
    """True when servo Z is aligned with body up (typical body→coxa yaw)."""
    return abs(axis_z.normalized().dot(up.normalized())) >= max(0.0, 1.0 - tol)


@dataclass
class MountCandidate:
    key: str
    frame: Frame3D
    component: str
    component_index: int
    file_path: str
    yaw_rad: float = 0.0
    radius_mm: float = 0.0


@dataclass
class MountAssignment:
    leg_id: int
    candidate: MountCandidate
    assignment: str
    coxa_axis_level_ok: bool
    yaw_value: float
    warnings: list[str]


def _angle_delta(a: float, b: float) -> float:
    """Smallest signed delta a-b in [-pi, pi]."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def select_anchor_index(
    candidates: list[MountCandidate],
    *,
    prefer_side: str,
) -> int:
    """Pick candidate with min |yaw|; tie-break by right-side preference."""
    best_i = 0
    best_abs = abs(candidates[0].yaw_rad)
    best_right = math.sin(candidates[0].yaw_rad)  # proxy for right_comp sign at unit circle

    for i, cand in enumerate(candidates[1:], start=1):
        abs_yaw = abs(cand.yaw_rad)
        right_proxy = math.sin(cand.yaw_rad)
        better = False
        if abs_yaw < best_abs - 1e-9:
            better = True
        elif abs(abs_yaw - best_abs) <= 1e-9:
            if prefer_side == "positive_right" and right_proxy > best_right:
                better = True
            elif prefer_side == "negative_right" and right_proxy < best_right:
                better = True
            elif prefer_side == "positive_left" and right_proxy < best_right:
                better = True
        if better:
            best_i = i
            best_abs = abs_yaw
            best_right = right_proxy
    return best_i


def auto_assign_mounts(
    candidates: list[MountCandidate],
    *,
    basis: BodyBasis,
    center: Vec3,
    radius_threshold_mm: float,
    sweep: str,
    prefer_side: str,
    leg_order_from_anchor: list[int],
    yaw_unit: str,
    coxa_up_dot_tol: float,
    expect_vertical: bool = True,
) -> tuple[list[MountAssignment], list[MountCandidate], list[str]]:
    """
    Filter by radius, sort by yaw, anchor, assign LegIDs.

    Returns (assignments, rejected, warnings).
    """
    warnings: list[str] = []
    rejected: list[MountCandidate] = []
    accepted: list[MountCandidate] = []

    for cand in candidates:
        yaw = yaw_angle_rad(cand.frame.origin, center, basis)
        radius = horizontal_radius(cand.frame.origin, center, basis)
        enriched = MountCandidate(
            key=cand.key,
            frame=cand.frame,
            component=cand.component,
            component_index=cand.component_index,
            file_path=cand.file_path,
            yaw_rad=yaw,
            radius_mm=radius,
        )
        if radius < radius_threshold_mm:
            logger.info(
                "mount_candidate_rejected key=%s radius=%.2f threshold=%.2f",
                enriched.key,
                radius,
                radius_threshold_mm,
            )
            rejected.append(enriched)
            continue
        accepted.append(enriched)

    if not accepted:
        warnings.append("auto mount: no candidates passed radius filter")
        return [], rejected, warnings

    reverse = sweep.lower() == "cw"
    accepted.sort(key=lambda c: c.yaw_rad, reverse=reverse)

    anchor_i = select_anchor_index(accepted, prefer_side=prefer_side)
    rotated = accepted[anchor_i:] + accepted[:anchor_i]

    if len(leg_order_from_anchor) < len(rotated):
        warnings.append(
            f"leg_order_from_anchor length {len(leg_order_from_anchor)} "
            f"< candidates {len(rotated)}; truncating"
        )

    assignments: list[MountAssignment] = []
    for i, cand in enumerate(rotated):
        if i >= len(leg_order_from_anchor):
            rejected.append(cand)
            warnings.append(f"extra mount candidate left unassigned: {cand.key}")
            continue
        leg_id = int(leg_order_from_anchor[i])
        if expect_vertical:
            level_ok = coxa_axis_vertical_ok(
                cand.frame.axis_z,
                basis.up,
                coxa_up_dot_tol,
            )
            bad_msg = (
                f"mount {cand.key} leg_id={leg_id}: coxa Z not aligned with body up "
                f"(|dot| < {1.0 - coxa_up_dot_tol})"
            )
        else:
            level_ok = coxa_axis_level_ok(
                cand.frame.axis_z,
                basis.up,
                coxa_up_dot_tol,
            )
            bad_msg = (
                f"mount {cand.key} leg_id={leg_id}: coxa Z not level vs body up "
                f"(|dot| > {coxa_up_dot_tol})"
            )
        mount_warnings: list[str] = []
        if not level_ok:
            logger.warning(bad_msg)
            mount_warnings.append(bad_msg)
            warnings.append(bad_msg)

        yaw_value = convert_yaw(cand.yaw_rad, yaw_unit)
        assignments.append(
            MountAssignment(
                leg_id=leg_id,
                candidate=cand,
                assignment="auto",
                coxa_axis_level_ok=level_ok,
                yaw_value=yaw_value,
                warnings=mount_warnings,
            )
        )
        logger.info(
            "mount_assigned key=%s leg_id=%s yaw=%.3f radius=%.2f assignment=auto",
            cand.key,
            leg_id,
            yaw_value,
            cand.radius_mm,
        )

    return assignments, rejected, warnings


def mean_origin(frames: list[Frame3D]) -> Vec3:
    if not frames:
        return Vec3(0.0, 0.0, 0.0)
    n = float(len(frames))
    return Vec3(
        sum(f.origin.x for f in frames) / n,
        sum(f.origin.y for f in frames) / n,
        sum(f.origin.z for f in frames) / n,
    )


def horizontal_extent_mm(points: list[Vec3], center: Vec3, basis: BodyBasis) -> float:
    if not points:
        return 0.0
    return max(horizontal_radius(p, center, basis) for p in points)
