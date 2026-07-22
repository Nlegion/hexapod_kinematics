"""Unit tests for config loader validation."""

from pathlib import Path

import pytest
import yaml

from hexapod_kinematics.core.config_loader import (
    ConfigError,
    default_config_dict,
    load_config,
    validate_config,
)


def test_default_config_valid() -> None:
    config = default_config_dict()
    validate_config(config)
    assert config["servo"]["mg996r_axis_length_mm"] == 42.9
    assert config["servo"]["body_height_mm"] == 42.9
    assert config["body_frame"]["yaw_unit"] == "deg"


def test_servo_out_of_range(tmp_path: Path) -> None:
    path = tmp_path / "bad.yml"
    path.write_text(
        yaml.dump({"servo": {"mg996r_axis_length_mm": 999}}),
        encoding="utf-8",
    )
    with pytest.raises(ConfigError, match="out of range"):
        load_config(path)


def test_yaml_override_merges(tmp_path: Path) -> None:
    path = tmp_path / "cfg.yml"
    path.write_text(
        yaml.dump(
            {
                "roots": {"cad": "D:/cad"},
                "lcs_role_map": {"CustomIn": "in"},
                "servo": {"femur_override_length_mm": 55.0},
            }
        ),
        encoding="utf-8",
    )
    config = load_config(path)
    assert config["roots"]["cad"] == "D:/cad"
    assert config["lcs_role_map"]["CustomIn"] == "in"
    assert config["lcs_role_map"]["LCS_in"] == "in"
    assert config["servo"]["femur_override_length_mm"] == 55.0


def test_leg_mount_map_invalid() -> None:
    config = default_config_dict()
    config["leg_mount_map"]["mount_leg_1"] = 9
    with pytest.raises(ConfigError, match="0..5"):
        validate_config(config)


def test_yaw_unit_invalid() -> None:
    config = default_config_dict()
    config["body_frame"]["yaw_unit"] = "grads"
    with pytest.raises(ConfigError, match="yaw_unit"):
        validate_config(config)
