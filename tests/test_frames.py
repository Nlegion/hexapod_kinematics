"""Unit tests for Mat4 frame transforms."""

from hexapod_kinematics.domain.frames import Frame3D, Mat4, Vec3


def test_identity_transform() -> None:
    frame = Frame3D(
        name="LCS",
        origin=Vec3(1, 2, 3),
        axis_x=Vec3(1, 0, 0),
        axis_y=Vec3(0, 1, 0),
        axis_z=Vec3(0, 0, 1),
    )
    out = Mat4.identity().transform_frame(frame)
    assert out.origin.as_list() == [1, 2, 3]


def test_translation() -> None:
    matrix = Mat4(
        rows=(
            (1.0, 0.0, 0.0, 10.0),
            (0.0, 1.0, 0.0, 20.0),
            (0.0, 0.0, 1.0, 30.0),
            (0.0, 0.0, 0.0, 1.0),
        )
    )
    point = matrix.transform_point(Vec3(1, 0, 0))
    assert point.as_list() == [11.0, 20.0, 30.0]
