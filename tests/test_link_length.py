"""Unit tests for link length and Z alignment."""

from hexapod_kinematics.domain.frames import Frame3D, Vec3
from hexapod_kinematics.domain.link_length import compute_link_length, synthesize_out


def _axis_frame(origin: Vec3, name: str = "LCS") -> Frame3D:
    return Frame3D(
        name=name,
        origin=origin,
        axis_x=Vec3(1, 0, 0),
        axis_y=Vec3(0, 1, 0),
        axis_z=Vec3(0, 0, 1),
    )


def test_length_along_z() -> None:
    fin = _axis_frame(Vec3(0, 0, 0), "in")
    fout = _axis_frame(Vec3(0, 0, 40), "out")
    result = compute_link_length(
        fin,
        fout,
        angle_tol_deg=0.1,
        lateral_tol_mm=0.5,
    )
    assert abs(result.length_mm - 40.0) < 1e-9
    assert result.axis_alignment_ok is True
    assert result.synthesized_out is False


def test_synthesize_out() -> None:
    fin = _axis_frame(Vec3(1, 2, 3), "in")
    synth = synthesize_out(fin, 40.7)
    assert synth.synthesized is True
    assert abs(synth.origin.z - (3 + 40.7)) < 1e-9

    result = compute_link_length(
        fin,
        None,
        angle_tol_deg=0.1,
        lateral_tol_mm=0.5,
        synthesize_length_mm=40.7,
    )
    assert result.synthesized_out is True
    assert abs(result.length_mm - 40.7) < 1e-9


def test_misaligned_warns() -> None:
    fin = _axis_frame(Vec3(0, 0, 0), "in")
    fout = _axis_frame(Vec3(10, 0, 40), "out")
    result = compute_link_length(
        fin,
        fout,
        angle_tol_deg=0.1,
        lateral_tol_mm=0.5,
    )
    assert result.axis_alignment_ok is False
    assert any("not aligned" in w for w in result.warnings)
    assert result.length_mm > 40


def test_missing_in() -> None:
    result = compute_link_length(
        None,
        _axis_frame(Vec3(0, 0, 1), "out"),
        angle_tol_deg=0.1,
        lateral_tol_mm=0.5,
    )
    assert "in" in result.missing
