"""Default folder -> kinematic role mapping for spider_body/996."""

from typing import Any

# folder name -> {role, mirror?, leaf_candidates, assembly_candidates}
DEFAULT_FOLDER_ROLES: dict[str, dict[str, Any]] = {
    "coxa_A_996": {
        "role": "coxa",
        "mirror": "A",
        "leaf_candidates": ["base.m3d", "body.m3d"],
    },
    "coxa_B_996": {
        "role": "coxa",
        "mirror": "B",
        "leaf_candidates": ["base.m3d", "body.m3d"],
    },
    "femur_A_996": {
        "role": "femur",
        "mirror": "A",
        "leaf_candidates": ["base.m3d", "middle.m3d"],
    },
    "femur_B_996": {
        "role": "femur",
        "mirror": "B",
        "leaf_candidates": ["base.m3d", "middle.m3d"],
    },
    "tiba_A_996": {
        "role": "tibia",
        "mirror": "A",
        "leaf_candidates": ["base.m3d", "body.m3d", "tiba_B_996.m3d"],
    },
    "tiba_B_996": {
        "role": "tibia",
        "mirror": "B",
        "leaf_candidates": ["base.m3d", "body.m3d", "tiba_B_996.m3d"],
    },
    "body_996": {
        "role": "body",
        "assembly_candidates": ["body_996.a3d", "body_test.a3d"],
        "leaf_hint": ["body_test_.m3d", "part_3.m3d"],
    },
}

LINK_ROLES = ("coxa", "femur", "tibia")
