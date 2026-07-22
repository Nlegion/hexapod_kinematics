"""Default body frame and mount auto-mapping settings."""

from typing import Any

DEFAULT_BODY_FRAME: dict[str, Any] = {
    # KOMPAS assembly axes → robot (996: long axis Z = forward, Y = up)
    "forward_axis": [0.0, 0.0, 1.0],
    "up_axis": [0.0, 1.0, 0.0],
    "yaw_unit": "deg",
    "coxa_up_dot_tol": 0.15,
    # Calculator world: X forward, Y left, Z up.
    # 996 labeled RIGHT mounts are at CAD +X → world -Y (see world_frame.py).
    "cad_to_world_rotation": [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    "body_forward_axis_world": [1.0, 0.0, 0.0],
    "body_up_axis_world": [0.0, 0.0, 1.0],
}

DEFAULT_BODY_MOUNT_AUTO: dict[str, Any] = {
    "enabled": True,
    "candidate_lcs_names": [
        "LCS_in",
        "mount_leg_1",
        "mount_leg_2",
        "mount_leg_3",
        "mount_leg_4",
        "mount_leg_5",
        "mount_leg_6",
    ],
    # fraction_of_extent | absolute_mm
    "radius_mode": "fraction_of_extent",
    "radius_fraction": 0.25,
    "radius_absolute_mm": 50.0,
    # cw matches FR→MR→RR→RL→ML→FL when forward=+Z, up=+Y
    "sweep": "cw",
    "start_anchor": {
        "prefer_side": "negative_right",
    },
    "leg_order_from_anchor": [0, 1, 2, 3, 4, 5],
}

LEG_NAMES = (
    "LEG_FRONT_RIGHT",
    "LEG_MIDDLE_RIGHT",
    "LEG_REAR_RIGHT",
    "LEG_REAR_LEFT",
    "LEG_MIDDLE_LEFT",
    "LEG_FRONT_LEFT",
)

MIN_USEFUL_REACH_MM = 20.0
