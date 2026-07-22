"""Merge YAML config with defaults and validate servo bounds."""

from __future__ import annotations

import copy
import logging
from pathlib import Path
from typing import Any

import yaml

from hexapod_kinematics.core.constants import (
    body_frame,
    folders,
    foot_pad,
    lcs,
    runtime,
    servo,
    tolerances,
)

logger = logging.getLogger(__name__)


class ConfigError(ValueError):
    """Invalid extractor configuration."""


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def default_config_dict() -> dict[str, Any]:
    return {
        "kompas": {
            "bin_path": str(runtime.DEFAULT_KOMPAS_BIN),
            "progid": runtime.KOMPAS_PROGID,
            "connect_retries": runtime.COM_CONNECT_RETRIES,
            "open_retries": runtime.OPEN_RETRIES,
            "retry_delay_sec": runtime.COM_RETRY_DELAY_SEC,
        },
        "roots": {
            "cad": str(runtime.DEFAULT_CAD_ROOT),
        },
        "folder_roles": copy.deepcopy(folders.DEFAULT_FOLDER_ROLES),
        "lcs_role_map": copy.deepcopy(lcs.DEFAULT_LCS_ROLE_MAP),
        "leg_mount_map": copy.deepcopy(lcs.DEFAULT_LEG_MOUNT_MAP),
        "mount_map_path": None,
        "servo": {
            "body_length_mm": servo.MG996R_BODY_LENGTH_MM,
            "body_width_mm": servo.MG996R_BODY_WIDTH_MM,
            "body_height_mm": servo.MG996R_BODY_HEIGHT_MM,
            "mg996r_axis_length_mm": servo.MG996R_AXIS_LENGTH_MM,
            "datasheet_ref": servo.DATASHEET_REF,
            "min_mm": servo.MG996R_AXIS_LENGTH_MIN_MM,
            "max_mm": servo.MG996R_AXIS_LENGTH_MAX_MM,
            "femur_override_length_mm": None,
        },
        "foot_pad": {
            "part": foot_pad.FOOT_PAD_PART,
            "diameter_mm": foot_pad.FOOT_PAD_DIAMETER_MM,
            "height_mm": foot_pad.FOOT_PAD_HEIGHT_MM,
            "recess_mm": foot_pad.FOOT_PAD_RECESS_MM,
            "protrusion_mm": foot_pad.FOOT_PAD_PROTRUSION_MM,
        },
        "body_frame": copy.deepcopy(body_frame.DEFAULT_BODY_FRAME),
        "body_mount_auto": copy.deepcopy(body_frame.DEFAULT_BODY_MOUNT_AUTO),
        "tolerances": {
            "angle_tol_deg": tolerances.ANGLE_TOL_DEG,
            "lateral_tol_mm": tolerances.LATERAL_TOL_MM,
        },
        "duplicate_lcs_policy": runtime.DUPLICATE_LCS_POLICY,
    }


def validate_config(config: dict[str, Any]) -> None:
    servo_cfg = config.get("servo", {})
    length = float(servo_cfg["mg996r_axis_length_mm"])
    min_mm = float(servo_cfg["min_mm"])
    max_mm = float(servo_cfg["max_mm"])
    if not (min_mm <= length <= max_mm):
        raise ConfigError(
            f"servo.mg996r_axis_length_mm={length} out of range "
            f"[{min_mm}, {max_mm}]"
        )
    override = servo_cfg.get("femur_override_length_mm")
    if override is not None:
        override_f = float(override)
        if not (min_mm <= override_f <= max_mm * 3):
            raise ConfigError(
                f"servo.femur_override_length_mm={override_f} out of sane range"
            )

    foot = config.get("foot_pad", {})
    for key in ("height_mm", "recess_mm", "protrusion_mm", "diameter_mm"):
        if key not in foot:
            continue
        val = float(foot[key])
        if val < 0:
            raise ConfigError(f"foot_pad.{key} must be >= 0")
    if "height_mm" in foot and "recess_mm" in foot:
        expected = float(foot["height_mm"]) - float(foot["recess_mm"])
        if "protrusion_mm" in foot and abs(float(foot["protrusion_mm"]) - expected) > 1e-6:
            raise ConfigError(
                "foot_pad.protrusion_mm must equal height_mm - recess_mm "
                f"(expected {expected})"
            )

    yaw_unit = config.get("body_frame", {}).get("yaw_unit", "deg")
    if yaw_unit not in ("deg", "rad"):
        raise ConfigError(f"body_frame.yaw_unit must be deg|rad, got {yaw_unit!r}")

    for axis_name in ("forward_axis", "up_axis"):
        axis = config.get("body_frame", {}).get(axis_name)
        if not isinstance(axis, list) or len(axis) != 3:
            raise ConfigError(f"body_frame.{axis_name} must be a 3-vector")

    auto = config.get("body_mount_auto", {})
    mode = auto.get("radius_mode", "fraction_of_extent")
    if mode not in ("fraction_of_extent", "absolute_mm"):
        raise ConfigError(f"unsupported body_mount_auto.radius_mode={mode!r}")
    sweep = str(auto.get("sweep", "cw")).lower()
    if sweep not in ("cw", "ccw"):
        raise ConfigError(f"body_mount_auto.sweep must be cw|ccw, got {sweep!r}")
    prefer = auto.get("start_anchor", {}).get("prefer_side", "negative_right")
    if prefer not in (
        "positive_right",
        "negative_right",
        "positive_left",
    ):
        raise ConfigError(f"unsupported start_anchor.prefer_side={prefer!r}")
    order = auto.get("leg_order_from_anchor", [])
    if not isinstance(order, list) or not order:
        raise ConfigError("body_mount_auto.leg_order_from_anchor must be non-empty list")
    if sorted(int(x) for x in order) != list(range(len(order))):
        # Allow non-0..n-1 only if values are unique in 0..5
        vals = [int(x) for x in order]
        if len(set(vals)) != len(vals) or any(v < 0 or v > 5 for v in vals):
            raise ConfigError(
                "leg_order_from_anchor must be unique LegIDs in 0..5"
            )

    policy = config.get("duplicate_lcs_policy")
    if policy != "first_with_warning":
        raise ConfigError(
            f"unsupported duplicate_lcs_policy={policy!r}; "
            "only 'first_with_warning' is implemented"
        )
    for name, leg_id in config.get("leg_mount_map", {}).items():
        if not isinstance(leg_id, int) or leg_id < 0 or leg_id > 5:
            raise ConfigError(
                f"leg_mount_map[{name!r}]={leg_id!r} must be int in 0..5"
            )


def load_mount_map_file(path: Path) -> dict[str, int]:
    """Load external mount map: key -> leg_id."""
    if not path.is_file():
        raise ConfigError(f"mount map file not found: {path}")
    with path.open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ConfigError("mount map root must be a mapping")
    result: dict[str, int] = {}
    for key, value in data.items():
        leg_id = int(value)
        if leg_id < 0 or leg_id > 5:
            raise ConfigError(f"mount map {key!r} -> {leg_id} out of 0..5")
        result[str(key)] = leg_id
    return result


def load_config(path: Path | None) -> dict[str, Any]:
    config = default_config_dict()
    config_path: Path | None = path
    if config_path is None:
        return config
    if not config_path.is_file():
        raise ConfigError(f"config file not found: {config_path}")
    with config_path.open(encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle) or {}
    if not isinstance(loaded, dict):
        raise ConfigError("config root must be a mapping")
    config = _deep_merge(config, loaded)
    validate_config(config)
    logger.info(
        "config_loaded",
        extra={"config_path": str(config_path.resolve())},
    )
    return config
