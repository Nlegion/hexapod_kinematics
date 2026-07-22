"""Read LocalCoordinateSystems from a KOMPAS part (API7)."""

from __future__ import annotations

import ctypes
import logging
from pathlib import Path
from typing import Any

from comtypes import automation
from comtypes.client import GetModule

from hexapod_kinematics.domain.frames import Frame3D, Vec3
from hexapod_kinematics.infrastructure.kompas.session import KompasError

logger = logging.getLogger(__name__)

_API7: Any | None = None


_DEFAULT_TLB = r"C:\Program Files\ASCON\KOMPAS-3D v21\Bin\kAPI7.tlb"


def _api7() -> Any:
    global _API7
    if _API7 is None:
        _API7 = GetModule(_DEFAULT_TLB)
    return _API7


def configure_tlb(bin_path: str | None) -> None:
    """Load kAPI7.tlb from KOMPAS Bin directory."""
    global _API7, _DEFAULT_TLB
    if bin_path:
        tlb = str(Path(bin_path) / "kAPI7.tlb")
        _DEFAULT_TLB = tlb
        _API7 = GetModule(tlb)
    else:
        _API7 = GetModule(_DEFAULT_TLB)


def _as_document3d(doc: Any) -> Any:
    api = _api7()
    try:
        return doc.QueryInterface(api.IKompasDocument3D)
    except Exception as exc:  # noqa: BLE001
        raise KompasError(f"document is not IKompasDocument3D: {exc}") from exc


def _get_top_part(doc: Any) -> Any:
    d3 = _as_document3d(doc)
    part = d3.TopPart
    if part is None:
        raise KompasError("TopPart is None")
    return part


def _get_lcs_collection(part: Any) -> Any:
    api = _api7()
    try:
        aux = part.QueryInterface(api.IAuxiliaryGeomContainer)
    except Exception as exc:  # noqa: BLE001
        raise KompasError(
            f"IAuxiliaryGeomContainer not available on part: {exc}"
        ) from exc
    collection = aux.LocalCoordinateSystems
    if collection is None:
        raise KompasError("LocalCoordinateSystems is None")
    return collection


def _get_vector(lcs: Any, axis: int) -> Vec3 | None:
    """Call ILocalCoordinateSystem.GetVector with proper out-args."""
    x = ctypes.c_double()
    y = ctypes.c_double()
    z = ctypes.c_double()
    ok = automation.VARIANT_BOOL()
    try:
        hr = lcs._ILocalCoordinateSystem__com_GetVector(
            axis,
            ctypes.byref(x),
            ctypes.byref(y),
            ctypes.byref(z),
            ctypes.byref(ok),
        )
    except Exception as exc:  # noqa: BLE001
        logger.warning("GetVector failed axis=%s: %s", axis, exc)
        return None
    if hr != 0 or not bool(ok):
        return None
    return Vec3(x.value, y.value, z.value).normalized()


def _axes_from_lcs(lcs: Any) -> tuple[Vec3, Vec3, Vec3, bool]:
    """
    Return (axis_x, axis_y, axis_z, axes_reliable).

    For OrientByObject LCS, GetVector may return the same direction for all
    axes; fall back to identity with axes_reliable=False.
    """
    vectors = [_get_vector(lcs, axis) for axis in range(3)]
    if any(v is None for v in vectors):
        logger.warning("LCS axes incomplete; using identity axes")
        return Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1), False

    axis_x, axis_y, axis_z = vectors  # type: ignore[misc]
    # Degenerate: all axes nearly parallel
    if (
        abs(axis_x.dot(axis_y)) > 0.99
        and abs(axis_x.dot(axis_z)) > 0.99
    ):
        logger.warning(
            "LCS GetVector axes degenerate (OrientByObject?); "
            "using identity axes"
        )
        return Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1), False
    return axis_x, axis_y, axis_z, True


def _frame_from_lcs_entity(entity: Any) -> Frame3D:
    api = _api7()
    lcs = entity.QueryInterface(api.ILocalCoordinateSystem)
    name = str(lcs.Name or "UnnamedLCS")
    origin = Vec3(float(lcs.X), float(lcs.Y), float(lcs.Z))
    axis_x, axis_y, axis_z, _reliable = _axes_from_lcs(lcs)
    return Frame3D(
        name=name,
        origin=origin,
        axis_x=axis_x,
        axis_y=axis_y,
        axis_z=axis_z,
    )


def _iter_collection(collection: Any) -> list[Any]:
    """API7 LocalCoordinateSystems.Item is 0-based."""
    items: list[Any] = []
    count = int(collection.Count)
    for index in range(count):
        item = collection.Item(index)
        if item is not None:
            items.append(item)
    return items


def read_local_coordinate_systems(doc: Any) -> list[Frame3D]:
    """Read all LCS from the document top part into Frame3D list."""
    part = _get_top_part(doc)
    collection = _get_lcs_collection(part)
    frames: list[Frame3D] = []
    for entity in _iter_collection(collection):
        try:
            frames.append(_frame_from_lcs_entity(entity))
        except Exception as exc:  # noqa: BLE001
            logger.warning("skip_lcs reason=%s", exc)
    logger.info("lcs_read count=%s names=%s", len(frames), [f.name for f in frames])
    return frames

