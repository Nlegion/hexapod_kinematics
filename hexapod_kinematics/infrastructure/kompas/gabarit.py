"""Optional part/assembly gabarit (bounding box) via API7."""

from __future__ import annotations

import logging
from typing import Any

from hexapod_kinematics.infrastructure.kompas.lcs_reader import _api7, _as_document3d
from hexapod_kinematics.infrastructure.kompas.session import KompasError

logger = logging.getLogger(__name__)


def read_gabarit(doc: Any) -> dict[str, Any] | None:
    """
    Try to read AABB of the top part.

    Returns dict with min/max/size in mm, or None if unsupported.
    """
    try:
        d3 = _as_document3d(doc)
        part = d3.TopPart
    except KompasError as exc:
        logger.warning("gabarit_top_part_failed: %s", exc)
        return None

    # Common patterns across KOMPAS API versions
    for getter_name in ("GetGabarit", "Gabarit", "GetBoundingBox"):
        if not hasattr(part, getter_name):
            continue
        try:
            getter = getattr(part, getter_name)
            raw = getter() if callable(getter) else getter
            parsed = _parse_gabarit(raw)
            if parsed is not None:
                return parsed
        except Exception as exc:  # noqa: BLE001
            logger.warning("gabarit_%s_failed: %s", getter_name, exc)

    # Some builds expose MathCalculation / Property
    try:
        api = _api7()
        if hasattr(part, "Property"):
            prop = part.Property
            if hasattr(prop, "GetGabarit"):
                parsed = _parse_gabarit(prop.GetGabarit())
                if parsed is not None:
                    return parsed
        _ = api  # keep import used
    except Exception as exc:  # noqa: BLE001
        logger.warning("gabarit_property_failed: %s", exc)

    logger.warning("gabarit_unavailable on TopPart")
    return None


def _parse_gabarit(raw: Any) -> dict[str, Any] | None:
    if raw is None:
        return None
    values = list(raw) if hasattr(raw, "__iter__") and not isinstance(raw, (str, bytes)) else None
    if values is None:
        return None
    if len(values) >= 6:
        mins = [float(values[0]), float(values[1]), float(values[2])]
        maxs = [float(values[3]), float(values[4]), float(values[5])]
        size = [maxs[i] - mins[i] for i in range(3)]
        return {"min_mm": mins, "max_mm": maxs, "size_mm": size}
    return None
