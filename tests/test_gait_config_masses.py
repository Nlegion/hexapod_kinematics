"""Gait YAML fail-fast and masses validation."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from hexapod_kinematics.domain.frames_world import mounts_from_model
from hexapod_kinematics.domain.gait_config import GaitConfigError, load_gait_config
from hexapod_kinematics.domain.masses import (
    load_masses_config,
    mass_weighted_com_xy,
    validate_masses,
)

ROOT = Path(__file__).resolve().parents[1]
GAIT = ROOT / "config" / "hexapod_gait.yml"
MASSES = ROOT / "config" / "masses_996.yml"
FIXTURE = ROOT / "tests" / "fixtures" / "kinematics_996_min.json"


def test_gait_load_ok() -> None:
    g = load_gait_config(GAIT)
    assert len(g["transfer_traj"]) == 4
    assert len(g["support_traj"]) == 4


def test_gait_fail_fast_missing_traj(tmp_path: Path) -> None:
    bad = {"neutral": 1500, "tripod_group_1": [0, 2, 4], "tripod_group_2": [1, 3, 5]}
    path = tmp_path / "bad.yml"
    path.write_text(yaml.safe_dump(bad), encoding="utf-8")
    with pytest.raises(GaitConfigError):
        load_gait_config(path)


def test_masses_sum_1800() -> None:
    data = load_masses_config(MASSES)
    b = validate_masses(data)
    assert abs(b.total_g - 1800.0) <= 1.0
    assert not b.warnings


def test_mass_com_source() -> None:
    import json

    model = json.loads(FIXTURE.read_text(encoding="utf-8"))
    mounts = mounts_from_model(model)
    masses = load_masses_config(MASSES)
    com, src = mass_weighted_com_xy(mounts=mounts, chains=None, masses=masses)
    assert src == "mass_model"
    assert com.shape == (2,)
