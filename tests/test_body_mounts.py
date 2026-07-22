"""Unit tests for body-frame yaw and auto mount assignment."""

import math

from hexapod_kinematics.domain.body_mounts import (
    MountCandidate,
    auto_assign_mounts,
    build_body_basis,
    convert_yaw,
    coxa_axis_level_ok,
    coxa_axis_vertical_ok,
    mean_origin,
)
from hexapod_kinematics.domain.frames import Frame3D, Vec3


def _frame(origin: Vec3, name: str = "LCS_in") -> Frame3D:
    return Frame3D(
        name=name,
        origin=origin,
        axis_x=Vec3(1, 0, 0),
        axis_y=Vec3(0, 0, 1),
        axis_z=Vec3(0, 1, 0),  # servo Z along body up — will fail level check
    )


def _level_frame(origin: Vec3, name: str = "LCS_in") -> Frame3D:
    # Body mount: servo Z along body up (yaw)
    return Frame3D(
        name=name,
        origin=origin,
        axis_x=Vec3(1, 0, 0),
        axis_y=Vec3(0, 0, 1),
        axis_z=Vec3(0, 1, 0),
    )


def test_yaw_unit_conversion() -> None:
    assert abs(convert_yaw(math.pi, "deg") - 180.0) < 1e-9
    assert abs(convert_yaw(math.pi / 2, "rad") - math.pi / 2) < 1e-9


def test_coxa_level_check() -> None:
    up = Vec3(0, 1, 0)
    assert coxa_axis_level_ok(Vec3(1, 0, 0), up, 0.15) is True
    assert coxa_axis_level_ok(Vec3(0, 1, 0), up, 0.15) is False
    assert coxa_axis_vertical_ok(Vec3(0, 1, 0), up, 0.15) is True
    assert coxa_axis_vertical_ok(Vec3(1, 0, 0), up, 0.15) is False


def test_auto_assign_six_points_996_geometry() -> None:
    """Fixture from body_996 unmapped LCS origins (assembly frame)."""
    origins = [
        Vec3(118.0, -2.5, 0.5),
        Vec3(-118.0, -2.5, 0.5),
        Vec3(-102.15, -2.5, -126.27),
        Vec3(104.27, -2.5, -127.15),
        Vec3(-104.27, -2.5, 127.15),
        Vec3(102.15, -2.5, 126.27),
    ]
    candidates = [
        MountCandidate(
            key=f"{i}:LCS_in",
            frame=_level_frame(origin),
            component="Det",
            component_index=i,
            file_path="base.m3d",
        )
        for i, origin in enumerate(origins)
    ]
    basis = build_body_basis(Vec3(0, 0, 1), Vec3(0, 1, 0))
    center = mean_origin([c.frame for c in candidates])
    assignments, rejected, warnings = auto_assign_mounts(
        candidates,
        basis=basis,
        center=center,
        radius_threshold_mm=50.0,
        sweep="cw",
        prefer_side="negative_right",
        leg_order_from_anchor=[0, 1, 2, 3, 4, 5],
        yaw_unit="deg",
        coxa_up_dot_tol=0.15,
        expect_vertical=True,
    )
    assert len(rejected) == 0
    assert len(assignments) == 6
    assert all(a.coxa_axis_level_ok for a in assignments)
    by_leg = {a.leg_id: a for a in assignments}
    assert set(by_leg) == {0, 1, 2, 3, 4, 5}
    # Opposite pairs roughly 180 deg apart
    for a, b in ((0, 3), (1, 4), (2, 5)):
        dyaw = abs(by_leg[a].yaw_value - by_leg[b].yaw_value)
        dyaw = min(dyaw, 360.0 - dyaw)
        assert abs(dyaw - 180.0) < 20.0, (a, b, dyaw)


def test_radius_rejects_center_point() -> None:
    basis = build_body_basis(Vec3(0, 0, 1), Vec3(0, 1, 0))
    candidates = [
        MountCandidate(
            key="0:LCS_in",
            frame=_level_frame(Vec3(0, 0, 0)),
            component="c",
            component_index=0,
            file_path="a.m3d",
        ),
        MountCandidate(
            key="1:LCS_in",
            frame=_level_frame(Vec3(100, 0, 0)),
            component="c",
            component_index=1,
            file_path="a.m3d",
        ),
    ]
    center = Vec3(0, 0, 0)
    assignments, rejected, _ = auto_assign_mounts(
        candidates,
        basis=basis,
        center=center,
        radius_threshold_mm=50.0,
        sweep="cw",
        prefer_side="negative_right",
        leg_order_from_anchor=[0],
        yaw_unit="deg",
        coxa_up_dot_tol=0.15,
    )
    assert len(rejected) == 1
    assert rejected[0].key == "0:LCS_in"
    assert len(assignments) == 1
