"""Unit tests for LCS role indexing and duplicate policy."""

from hexapod_kinematics.domain.frames import Frame3D, Vec3
from hexapod_kinematics.domain.lcs_roles import index_frames_by_role


def _frame(name: str) -> Frame3D:
    return Frame3D(
        name=name,
        origin=Vec3(0, 0, 0),
        axis_x=Vec3(1, 0, 0),
        axis_y=Vec3(0, 1, 0),
        axis_z=Vec3(0, 0, 1),
    )


def test_alias_roles() -> None:
    role_map = {"LCS_": "in", "LCS__": "out"}
    by_role, warnings = index_frames_by_role(
        [_frame("LCS_"), _frame("LCS__")],
        role_map,
    )
    assert "in" in by_role
    assert "out" in by_role
    assert warnings == []


def test_duplicate_name_first_with_warning() -> None:
    f1 = Frame3D(
        name="LCS_in",
        origin=Vec3(0, 0, 0),
        axis_x=Vec3(1, 0, 0),
        axis_y=Vec3(0, 1, 0),
        axis_z=Vec3(0, 0, 1),
    )
    f2 = Frame3D(
        name="LCS_in",
        origin=Vec3(1, 0, 0),
        axis_x=Vec3(1, 0, 0),
        axis_y=Vec3(0, 1, 0),
        axis_z=Vec3(0, 0, 1),
    )
    by_role, warnings = index_frames_by_role(
        [f1, f2],
        {"LCS_in": "in"},
    )
    assert by_role["in"].origin.x == 0
    assert by_role["in"].ambiguous is True
    assert any("duplicate" in w for w in warnings)


def test_unknown_name_ignored() -> None:
    by_role, warnings = index_frames_by_role(
        [_frame("Other")],
        {"LCS_in": "in"},
    )
    assert by_role == {}
    assert warnings == []
