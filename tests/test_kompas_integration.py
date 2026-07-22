"""Optional KOMPAS integration tests (manual / non-CI)."""

from pathlib import Path

import pytest

from hexapod_kinematics.core.config_loader import load_config
from hexapod_kinematics.infrastructure.kompas.documents import close_document, open_document
from hexapod_kinematics.infrastructure.kompas.lcs_reader import (
    configure_tlb,
    read_local_coordinate_systems,
)
from hexapod_kinematics.infrastructure.kompas.session import KompasSession

pytestmark = pytest.mark.kompas


@pytest.fixture
def config() -> dict:
    return load_config(Path("extractor_config.yml"))


@pytest.fixture
def cad_base(config: dict) -> Path:
    root = Path(config["roots"]["cad"])
    base = root / "coxa_A_996" / "base.m3d"
    if not base.is_file():
        pytest.skip(f"CAD file missing: {base}")
    return base


def test_read_lcs_from_base_m3d(config: dict, cad_base: Path) -> None:
    kompas = config["kompas"]
    configure_tlb(str(kompas["bin_path"]))
    with KompasSession(
        progid=kompas["progid"],
        connect_retries=int(kompas["connect_retries"]),
        retry_delay_sec=float(kompas["retry_delay_sec"]),
    ) as session:
        doc = open_document(session, cad_base)
        try:
            frames = read_local_coordinate_systems(doc)
        finally:
            close_document(doc)
    assert len(frames) >= 1
    names = {frame.name for frame in frames}
    assert names & {"LCS_in", "LCS_out", "LCS_", "LCS__", "in", "out"}
    by_name = {frame.name: frame for frame in frames}
    if "LCS_in" in by_name and "LCS_out" in by_name:
        delta = by_name["LCS_out"].origin - by_name["LCS_in"].origin
        assert delta.norm() > 1.0


def test_body_mounts_six_and_symmetry(config: dict) -> None:
    from hexapod_kinematics.application.extract_body import extract_body_mounts
    from hexapod_kinematics.domain.body_mounts import mean_origin

    kompas = config["kompas"]
    cad_root = Path(config["roots"]["cad"])
    configure_tlb(str(kompas["bin_path"]))
    with KompasSession(
        progid=kompas["progid"],
        connect_retries=int(kompas["connect_retries"]),
        retry_delay_sec=float(kompas["retry_delay_sec"]),
    ) as session:
        body = extract_body_mounts(session, cad_root, config)

    assert len(body.mounts) == 6, body.warnings
    by_leg = {m.leg_id: m for m in body.mounts}
    assert set(by_leg) == {0, 1, 2, 3, 4, 5}
    center = mean_origin([m.frame for m in body.mounts])
    for a, b in ((0, 3), (1, 4), (2, 5)):
        ra = (by_leg[a].frame.origin - center).norm()
        rb = (by_leg[b].frame.origin - center).norm()
        assert abs(ra - rb) / max(ra, rb) < 0.05
        dyaw = abs(by_leg[a].yaw_value - by_leg[b].yaw_value)
        dyaw = min(dyaw, 360.0 - dyaw)
        assert abs(dyaw - 180.0) < 15.0
