"""Load and validate hexapod_gait.yml (fail-fast on missing traj)."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


class GaitConfigError(ValueError):
    """Invalid or incomplete gait configuration."""


REQUIRED_KEYS = (
    "transfer_traj",
    "support_traj",
    "tripod_group_1",
    "tripod_group_2",
    "neutral",
)


def load_gait_config(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise GaitConfigError(f"gait config not found: {path}")
    with path.open(encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise GaitConfigError(f"gait config must be a mapping: {path}")

    missing = [k for k in REQUIRED_KEYS if k not in data]
    if missing:
        raise GaitConfigError(
            f"gait config missing required keys {missing}. "
            "Pulse replay needs transfer_traj/support_traj as µs arrays "
            "[coxa, femur, tibia] (snapshot of Config.h), not XY offsets."
        )

    transfer = data["transfer_traj"]
    support = data["support_traj"]
    if not isinstance(transfer, list) or not isinstance(support, list):
        raise GaitConfigError("transfer_traj and support_traj must be lists")
    if len(transfer) == 0 or len(support) == 0:
        raise GaitConfigError("transfer_traj/support_traj must be non-empty")
    if len(transfer) != len(support):
        raise GaitConfigError(
            f"transfer_traj length {len(transfer)} != support_traj {len(support)}"
        )

    traj_steps = int(data.get("traj_steps", len(transfer)))
    if len(transfer) != traj_steps:
        raise GaitConfigError(
            f"transfer_traj length {len(transfer)} != traj_steps {traj_steps}"
        )

    for name, traj in (("transfer_traj", transfer), ("support_traj", support)):
        for i, row in enumerate(traj):
            if not isinstance(row, (list, tuple)) or len(row) != 3:
                raise GaitConfigError(
                    f"{name}[{i}] must be [coxa, femur, tibia] µs, got {row!r}"
                )
            for j, val in enumerate(row):
                if not isinstance(val, (int, float)):
                    raise GaitConfigError(f"{name}[{i}][{j}] must be numeric µs")

    for key in ("tripod_group_1", "tripod_group_2"):
        group = data[key]
        if not isinstance(group, list) or len(group) != 3:
            raise GaitConfigError(f"{key} must be a list of 3 leg ids")
        for lid in group:
            if int(lid) not in range(6):
                raise GaitConfigError(f"{key} contains invalid leg_id {lid}")

    sna = data.get("servo_neutral_angles")
    if sna is not None:
        if not isinstance(sna, (list, tuple)) or len(sna) != 3:
            raise GaitConfigError(
                "servo_neutral_angles must be [coxa, femur, tibia] radians"
            )
        for j, val in enumerate(sna):
            if not isinstance(val, (int, float)):
                raise GaitConfigError(f"servo_neutral_angles[{j}] must be numeric rad")

    if "ik_reach_margin_mm" in data and data["ik_reach_margin_mm"] is not None:
        if not isinstance(data["ik_reach_margin_mm"], (int, float)):
            raise GaitConfigError("ik_reach_margin_mm must be numeric")

    data.setdefault("traj_steps", len(transfer))
    data.setdefault("ik_reach_margin_mm", 5.0)
    data["_source_path"] = str(path.resolve())
    return data


def assert_matches_firmware_snapshot(gait: dict[str, Any]) -> list[str]:
    """Return warnings if YAML drifts from known Config.h snapshot."""
    warnings: list[str] = []
    expected_transfer = [
        [1320, 1700, 1250],
        [1500, 1800, 1150],
        [1680, 1800, 1150],
        [1680, 1550, 1450],
    ]
    expected_support = [
        [1680, 1550, 1450],
        [1600, 1520, 1480],
        [1450, 1490, 1510],
        [1320, 1470, 1530],
    ]
    if [list(map(int, row)) for row in gait["transfer_traj"]] != expected_transfer:
        warnings.append(
            "transfer_traj differs from Config.h snapshot — re-sync hexapod_gait.yml"
        )
    if [list(map(int, row)) for row in gait["support_traj"]] != expected_support:
        warnings.append(
            "support_traj differs from Config.h snapshot — re-sync hexapod_gait.yml"
        )
    if int(gait.get("neutral", -1)) != 1500:
        warnings.append("neutral != 1500 (Config.h NEUTRAL)")
    return warnings
