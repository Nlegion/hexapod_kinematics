"""Path / candidate helpers for body mount extraction (keeps extract_body thin)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from hexapod_kinematics.domain.frames import Frame3D
from hexapod_kinematics.domain.lcs_roles import resolve_role


def pick_assembly(folder: Path, candidates: list[str]) -> Path | None:
    for name in candidates:
        path = folder / name
        if path.is_file():
            return path
    a3d = sorted(folder.glob("*.a3d"))
    return a3d[0] if a3d else None


def resolve_component_path(
    cad_root: Path,
    body_folder: Path,
    file_path: Path | None,
    component_name: str,
) -> Path | None:
    if file_path is not None and file_path.is_file():
        return file_path
    guesses: list[Path] = []
    if file_path is not None:
        guesses.append(body_folder / file_path.name)
        guesses.append(cad_root / file_path.name)
    guesses.append(body_folder / f"{component_name}.m3d")
    for path in guesses:
        if path.is_file():
            return path
    for path in body_folder.rglob("*.m3d"):
        if (
            path.stem.lower() in component_name.lower()
            or component_name.lower() in path.stem.lower()
        ):
            return path
    return None


def leg_id_for_lcs(
    name: str,
    role_map: dict[str, str],
    leg_mount_map: dict[str, int],
) -> int | None:
    if name in leg_mount_map:
        return int(leg_mount_map[name])
    role = resolve_role(name, role_map)
    if role and role.startswith("mount_leg_"):
        try:
            index = int(role.rsplit("_", maxsplit=1)[-1])
            return index - 1
        except ValueError:
            return None
    return None


def candidate_key(component_index: int, lcs_name: str) -> str:
    return f"{component_index}:{lcs_name}"


def payload_from_frame(
    frame: Frame3D,
    *,
    component: str,
    component_index: int,
    part_path: Path,
) -> dict[str, Any]:
    return {
        "name": frame.name,
        "component": component,
        "component_index": component_index,
        "key": candidate_key(component_index, frame.name),
        "file": str(part_path),
        "origin_mm": frame.origin.as_list(),
        "axis_x": frame.axis_x.as_list(),
        "axis_y": frame.axis_y.as_list(),
        "axis_z": frame.axis_z.as_list(),
        "ambiguous": frame.ambiguous,
    }
