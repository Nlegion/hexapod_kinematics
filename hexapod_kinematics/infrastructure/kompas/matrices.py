"""Assembly component placement matrices (API7)."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from hexapod_kinematics.domain.frames import Mat4, Vec3
from hexapod_kinematics.infrastructure.kompas.lcs_reader import _api7
from hexapod_kinematics.infrastructure.kompas.session import KompasError

logger = logging.getLogger(__name__)


@dataclass
class AssemblyComponent:
    name: str
    file_path: Path | None
    placement: Mat4


def _triplet(values: Any) -> tuple[float, float, float]:
    if values is None:
        raise KompasError("empty triplet")
    items = list(values)
    return float(items[0]), float(items[1]), float(items[2])


def _matrix_from_placement3d(placement: Any) -> Mat4:
    """Build Mat4 from IPlacement3D GetOrigin + GetVector(0..2)."""
    if placement is None:
        return Mat4.identity()
    origin = _triplet(placement.GetOrigin())
    axis_x = Vec3(*_triplet(placement.GetVector(0))).normalized()
    axis_y = Vec3(*_triplet(placement.GetVector(1))).normalized()
    axis_z = Vec3(*_triplet(placement.GetVector(2))).normalized()
    return Mat4(
        rows=(
            (axis_x.x, axis_y.x, axis_z.x, origin[0]),
            (axis_x.y, axis_y.y, axis_z.y, origin[1]),
            (axis_x.z, axis_y.z, axis_z.z, origin[2]),
            (0.0, 0.0, 0.0, 1.0),
        )
    )


def iter_assembly_components(doc: Any) -> list[AssemblyComponent]:
    """Enumerate first-level components of an assembly (0-based Parts)."""
    api = _api7()
    d3 = doc.QueryInterface(api.IKompasDocument3D)
    top = d3.TopPart
    parts_coll = top.Parts
    if parts_coll is None:
        raise KompasError("TopPart.Parts is None")

    count = int(parts_coll.Count)
    components: list[AssemblyComponent] = []
    for index in range(count):
        obj = parts_coll.Item(index)
        if obj is None:
            continue
        try:
            part = obj.QueryInterface(api.IPart7)
        except Exception as exc:  # noqa: BLE001
            logger.warning("part_qi_failed index=%s: %s", index, exc)
            continue
        name = str(part.Name or f"Part_{index}")
        file_name = str(part.FileName or "") or None
        path = Path(file_name) if file_name else None
        placement = _matrix_from_placement3d(part.Placement)
        components.append(
            AssemblyComponent(name=name, file_path=path, placement=placement)
        )
    logger.info("assembly_components count=%s", len(components))
    return components
