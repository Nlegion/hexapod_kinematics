"""
World frame helpers for the motion calculator.

Parent frame (world / body):
  X = forward, Y = left, Z = up

CAD assembly (996) is transformed with cad_to_world_rotation:
  p_world = R @ p_cad

Coxa-local (firmware IK): after mount transform, x/y horizontal, z = up.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from hexapod_kinematics.core.constants import world_frame as wf


def cad_to_world_matrix(model_or_meta: dict[str, Any] | None = None) -> np.ndarray:
    """3x3 rotation; columns are CAD basis vectors expressed in world."""
    meta = model_or_meta or {}
    if "meta" in meta:
        meta = meta.get("meta") or {}
    body = meta.get("body_frame") or {}
    raw = body.get("cad_to_world_rotation") or wf.DEFAULT_CAD_TO_WORLD_ROTATION
    r = np.asarray(raw, dtype=float)
    if r.shape != (3, 3):
        raise ValueError(f"cad_to_world_rotation must be 3x3, got {r.shape}")
    return r


def rotate_cad_to_world(vec_cad: np.ndarray, r: np.ndarray) -> np.ndarray:
    v = np.asarray(vec_cad, dtype=float).reshape(3)
    return r @ v


@dataclass(frozen=True, slots=True)
class MountWorld:
    """Leg mount expressed in calculator world frame."""

    leg_id: int
    leg_name: str
    origin: np.ndarray  # (3,)
    axis_x: np.ndarray
    axis_y: np.ndarray
    axis_z: np.ndarray
    yaw_deg: float

    @property
    def rotation(self) -> np.ndarray:
        """3x3 with columns = mount axes in world (Z = servo)."""
        return np.column_stack([self.axis_x, self.axis_y, self.axis_z])


def _reorient_leg_frame(mount: MountWorld, body_center: np.ndarray) -> MountWorld:
    """
    Build coxa kinematics frame with X = horizontal outward from body.

    CAD LCS X/Y often follow the part, not the leg reach direction — using them
    raw makes neutral coxa point sideways/inward (mirrored look). Firmware IK
    zero is along the coxa link; we align that with radial-out for viz + IK.
    Z stays ≈ world up (servo yaw axis).
    """
    z = np.asarray(mount.axis_z, dtype=float).copy()
    if z[2] < 0.0:
        z = -z
    zn = np.linalg.norm(z)
    z = z / zn if zn > 1e-9 else np.array([0.0, 0.0, 1.0])

    radial = np.asarray(mount.origin, dtype=float) - np.asarray(body_center, dtype=float)
    radial[2] = 0.0
    rn = np.linalg.norm(radial)
    if rn < 1e-9:
        # fallback: project CAD axis_x onto horizontal
        x = np.asarray(mount.axis_x, dtype=float).copy()
        x[2] = 0.0
        xn = np.linalg.norm(x)
        x = x / xn if xn > 1e-9 else np.array([1.0, 0.0, 0.0])
    else:
        x = radial / rn

    y = np.cross(z, x)
    yn = np.linalg.norm(y)
    if yn < 1e-9:
        y = np.array([0.0, 1.0, 0.0])
    else:
        y = y / yn
    # re-orthogonalize x in case z wasn't exactly vertical
    x = np.cross(y, z)
    x = x / (np.linalg.norm(x) + 1e-12)

    return MountWorld(
        leg_id=mount.leg_id,
        leg_name=mount.leg_name,
        origin=np.asarray(mount.origin, dtype=float).copy(),
        axis_x=x,
        axis_y=y,
        axis_z=z,
        yaw_deg=mount.yaw_deg,
    )


def mounts_from_model(model: dict[str, Any]) -> list[MountWorld]:
    """Convert CAD body_mounts to world-frame mounts (sorted by leg_id).

    Orient each leg so local +X points horizontally outward from the body
    center (neutral coxa reach), not raw CAD LCS X.
    """
    r = cad_to_world_matrix(model)
    raw: list[MountWorld] = []
    for m in sorted(model["body_mounts"], key=lambda x: int(x["leg_id"])):
        raw.append(
            MountWorld(
                leg_id=int(m["leg_id"]),
                leg_name=str(m.get("leg_name", "")),
                origin=rotate_cad_to_world(m["origin_mm"], r),
                axis_x=rotate_cad_to_world(m["axis_x"], r),
                axis_y=rotate_cad_to_world(m["axis_y"], r),
                axis_z=rotate_cad_to_world(m["axis_z"], r),
                yaw_deg=float(m.get("yaw", 0.0)),
            )
        )
    if not raw:
        return raw
    center = np.mean(np.stack([m.origin for m in raw], axis=0), axis=0)
    return [_reorient_leg_frame(m, center) for m in raw]


def place_hips_above_ground(
    mounts: list[MountWorld],
    body_height_mm: float,
) -> list[MountWorld]:
    """
    Set hip Z = body_height so the body floats above ground plane z=0.

    This does **not** IK-fit tips to the floor. For stance tip_z ≈ 0 use mode
    ``pulse_aligned``: same joint angles, rigid body ΔZ so min(stance tip_z)→0.
    Combined with this hip placement, invariants expect stance tips near z=0
    after alignment (±1 mm).
    """
    h = float(body_height_mm)
    out: list[MountWorld] = []
    for m in mounts:
        origin = np.asarray(m.origin, dtype=float).copy()
        origin[2] = h
        out.append(
            MountWorld(
                leg_id=m.leg_id,
                leg_name=m.leg_name,
                origin=origin,
                axis_x=np.asarray(m.axis_x, dtype=float).copy(),
                axis_y=np.asarray(m.axis_y, dtype=float).copy(),
                axis_z=np.asarray(m.axis_z, dtype=float).copy(),
                yaw_deg=m.yaw_deg,
            )
        )
    return out


def resolve_body_height_mm(
    gait: dict[str, Any],
    model: dict[str, Any],
) -> float:
    """Explicit gait body_height, else gabarit-based, else fallback."""
    explicit = gait.get("body_height_mm")
    if explicit is not None:
        return float(explicit)

    clearance = float(gait.get("ground_clearance_mm", 5.0))
    r = cad_to_world_matrix(model)

    # Prefer assembly gabarit if present
    gabarits = model.get("gabarits") or {}
    for key, g in gabarits.items():
        if not isinstance(g, dict):
            continue
        if "body" not in str(key).lower() and "assembly" not in str(key).lower():
            continue
        size = g.get("size_mm") or g.get("dimensions_mm")
        if size and len(size) >= 3:
            extent_world = np.abs(r @ np.asarray(size, dtype=float))
            return float(0.5 * extent_world[2] + clearance)

    meta_size = (model.get("meta") or {}).get("body_size_mm")
    if meta_size and len(meta_size) >= 3:
        extent_world = np.abs(r @ np.asarray(meta_size, dtype=float))
        return float(0.5 * extent_world[2] + clearance)

    return float(gait.get("body_height_fallback_mm", 45.0))
