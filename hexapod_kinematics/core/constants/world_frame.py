"""Calculator world frame: X forward, Y left, Z up."""

from __future__ import annotations

from typing import Any

# p_world = R @ p_cad  (columns = CAD basis expressed in world)
# 996 mounts: RIGHT legs sit at CAD +X, LEFT at CAD −X (export labels).
# World: X forward, Y left, Z up ⇒ CAD +X → world −Y (right), CAD +Z → +X, CAD +Y → +Z.
DEFAULT_CAD_TO_WORLD_ROTATION: list[list[float]] = [
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
]

DEFAULT_BODY_FORWARD_AXIS_WORLD: list[float] = [1.0, 0.0, 0.0]
DEFAULT_BODY_UP_AXIS_WORLD: list[float] = [0.0, 0.0, 1.0]

LEG_SHORT_NAMES: tuple[str, ...] = ("FR", "MR", "RR", "RL", "ML", "FL")

# front / mid / hind bands for Microvelia-style logging
LEG_BAND: dict[int, str] = {
    0: "front",  # FR
    1: "mid",  # MR
    2: "hind",  # RR
    3: "hind",  # RL
    4: "mid",  # ML
    5: "front",  # FL
}


def default_world_meta() -> dict[str, Any]:
    return {
        "cad_to_world_rotation": [row[:] for row in DEFAULT_CAD_TO_WORLD_ROTATION],
        "body_forward_axis": list(DEFAULT_BODY_FORWARD_AXIS_WORLD),
        "body_up_axis": list(DEFAULT_BODY_UP_AXIS_WORLD),
    }
