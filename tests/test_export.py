"""Unit tests for JSON model and C header export."""

from pathlib import Path

from hexapod_kinematics.application.build_model import build_kinematics_model
from hexapod_kinematics.application.export_header import (
    render_kinematics_header,
    write_kinematics_header,
)
from hexapod_kinematics.application.export_json import write_kinematics_json
from hexapod_kinematics.application.extract_body import BodyExtraction, BodyMount
from hexapod_kinematics.application.extract_links import LinkExtraction
from hexapod_kinematics.core.config_loader import default_config_dict
from hexapod_kinematics.domain.frames import Frame3D, Vec3
from hexapod_kinematics.domain.link_length import LinkLengthResult


def _result(length: float, *, synthesized: bool = False) -> LinkLengthResult:
    fin = Frame3D(
        name="in",
        origin=Vec3(0, 0, 0),
        axis_x=Vec3(1, 0, 0),
        axis_y=Vec3(0, 1, 0),
        axis_z=Vec3(0, 0, 1),
    )
    fout = Frame3D(
        name="out",
        origin=Vec3(0, 0, length),
        axis_x=Vec3(1, 0, 0),
        axis_y=Vec3(0, 1, 0),
        axis_z=Vec3(0, 0, 1),
        synthesized=synthesized,
    )
    return LinkLengthResult(
        length_mm=length,
        axis_alignment_ok=True,
        angle_deg=0.0,
        lateral_mm=0.0,
        synthesized_out=synthesized,
        ambiguous=False,
        frame_in=fin,
        frame_out=fout,
    )


def test_build_model_averages_and_femur_flag() -> None:
    config = default_config_dict()
    links = [
        LinkExtraction(
            folder="coxa_A_996",
            role="coxa",
            mirror="A",
            source_path=Path("a.m3d"),
            sha256="aa",
            result=_result(50.0),
        ),
        LinkExtraction(
            folder="coxa_B_996",
            role="coxa",
            mirror="B",
            source_path=Path("b.m3d"),
            sha256="bb",
            result=_result(60.0),
        ),
        LinkExtraction(
            folder="femur_A_996",
            role="femur",
            mirror="A",
            source_path=Path("f.m3d"),
            sha256="ff",
            result=_result(42.9, synthesized=True),
        ),
        LinkExtraction(
            folder="femur_B_996",
            role="femur",
            mirror="B",
            source_path=Path("g.m3d"),
            sha256="gg",
            result=_result(42.9, synthesized=True),
        ),
        LinkExtraction(
            folder="tiba_A_996",
            role="tibia",
            mirror="A",
            source_path=Path("t.m3d"),
            sha256="tt",
            result=_result(52.5),
        ),
    ]
    model = build_kinematics_model(
        config=config,
        config_path=None,
        links=links,
        body=BodyExtraction(assembly_path=None, sha256=None),
    )
    assert model["lengths_mm"]["coxa"] == 55.0
    assert model["lengths_mm"]["femur"] == 42.9
    assert model["lengths_mm"]["tibia"] == 52.5
    assert model["lengths_mm"]["tibia_effective"] == 54.5
    assert model["femur_synthesized_approximate"] is True
    assert model["femur_source"] == "datasheet_height"
    assert model["servo_datasheet"]["body_height_mm"] == 42.9
    assert model["foot_pad"]["part"] == "RF-F12050"
    assert model["foot_pad"]["protrusion_mm"] == 2.0
    assert model["reach"]["max_reach_mm"] == 42.9 + 54.5


def test_femur_override() -> None:
    config = default_config_dict()
    config["servo"]["femur_override_length_mm"] = 70.0
    links = [
        LinkExtraction(
            folder="femur_A_996",
            role="femur",
            mirror="A",
            source_path=Path("f.m3d"),
            sha256="ff",
            result=_result(42.9, synthesized=True),
        ),
    ]
    model = build_kinematics_model(
        config=config,
        config_path=None,
        links=links,
        body=BodyExtraction(assembly_path=None, sha256=None),
    )
    assert model["lengths_mm"]["femur"] == 70.0
    assert model["femur_source"] == "override"
    assert model["femur_synthesized_approximate"] is False


def test_header_contains_mounts_and_axes() -> None:
    frame = Frame3D(
        name="LCS_in",
        origin=Vec3(1, 2, 3),
        axis_x=Vec3(1, 0, 0),
        axis_y=Vec3(0, 1, 0),
        axis_z=Vec3(0, 0, 1),
    )
    body = BodyExtraction(assembly_path=None, sha256=None)
    body.mounts.append(
        BodyMount(
            leg_id=0,
            lcs_name="LCS_in",
            component="c",
            component_index=0,
            frame=frame,
            assignment="auto",
            yaw_value=12.5,
            coxa_axis_level_ok=True,
            key="0:LCS_in",
        )
    )
    model = build_kinematics_model(
        config=default_config_dict(),
        config_path=None,
        links=[],
        body=body,
    )
    model["lengths_mm"] = {"coxa": 52.5, "femur": 42.9, "tibia": 52.5, "tibia_effective": 54.5}
    model["femur_synthesized_approximate"] = True
    model["foot_pad"] = {"protrusion_mm": 2.0, "part": "RF-F12050"}
    text = render_kinematics_header(model)
    assert text.startswith("#pragma once")
    assert "LEG_MOUNT_ORIGIN" in text
    assert "LEG_MOUNT_YAW_DEG" in text
    assert "LEG_MOUNT_AXES" in text
    assert "approx: MG996R height" in text
    assert "FOOT_PAD_PROTRUSION_MM" in text
    assert "TIBIA_EFFECTIVE_LENGTH" in text


def test_write_artifacts(tmp_path: Path) -> None:
    model = {
        "meta": {
            "generated_at": "2026-07-22T00:00:00Z",
            "tool_version": "0.1.0",
            "yaw_unit": "deg",
        },
        "lengths_mm": {"coxa": 1.0, "femur": 2.0, "tibia": 3.0},
        "femur_synthesized_approximate": False,
        "femur_source": "measured",
        "servo_datasheet": {"axis_length_mm": 42.9},
        "links": [],
        "body_mounts": [],
        "unmapped_lcs": [],
        "warnings": [],
        "sources": [],
    }
    json_path = write_kinematics_json(model, tmp_path / "kinematics_996.json")
    header_path = write_kinematics_header(
        model,
        tmp_path / "generated_kinematics_996.h",
    )
    assert json_path.is_file()
    assert "COXA_LENGTH" in header_path.read_text(encoding="utf-8")
