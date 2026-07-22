"""3D frames and vector helpers for LCS math."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Vec3:
    x: float
    y: float
    z: float

    def __add__(self, other: Vec3) -> Vec3:
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Vec3) -> Vec3:
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> Vec3:
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    def dot(self, other: Vec3) -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: Vec3) -> Vec3:
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    def norm(self) -> float:
        return math.sqrt(self.dot(self))

    def normalized(self) -> Vec3:
        length = self.norm()
        if length == 0.0:
            return Vec3(0.0, 0.0, 0.0)
        return self * (1.0 / length)

    def as_list(self) -> list[float]:
        return [self.x, self.y, self.z]

    @staticmethod
    def from_list(values: list[float] | tuple[float, ...]) -> Vec3:
        return Vec3(float(values[0]), float(values[1]), float(values[2]))


@dataclass(frozen=True, slots=True)
class Frame3D:
    """Local coordinate system: origin + orthonormal axes (Z = servo axis)."""

    name: str
    origin: Vec3
    axis_x: Vec3
    axis_y: Vec3
    axis_z: Vec3
    ambiguous: bool = False
    synthesized: bool = False

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "origin_mm": self.origin.as_list(),
            "axis_x": self.axis_x.as_list(),
            "axis_y": self.axis_y.as_list(),
            "axis_z": self.axis_z.as_list(),
            "ambiguous": self.ambiguous,
            "synthesized": self.synthesized,
        }


@dataclass(frozen=True, slots=True)
class Mat4:
    """Row-major 4x4 affine transform (rotation + translation)."""

    rows: tuple[
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
    ]

    @staticmethod
    def identity() -> Mat4:
        return Mat4(
            rows=(
                (1.0, 0.0, 0.0, 0.0),
                (0.0, 1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
            )
        )

    def transform_point(self, point: Vec3) -> Vec3:
        r = self.rows
        return Vec3(
            r[0][0] * point.x + r[0][1] * point.y + r[0][2] * point.z + r[0][3],
            r[1][0] * point.x + r[1][1] * point.y + r[1][2] * point.z + r[1][3],
            r[2][0] * point.x + r[2][1] * point.y + r[2][2] * point.z + r[2][3],
        )

    def transform_direction(self, direction: Vec3) -> Vec3:
        r = self.rows
        return Vec3(
            r[0][0] * direction.x + r[0][1] * direction.y + r[0][2] * direction.z,
            r[1][0] * direction.x + r[1][1] * direction.y + r[1][2] * direction.z,
            r[2][0] * direction.x + r[2][1] * direction.y + r[2][2] * direction.z,
        )

    def transform_frame(self, frame: Frame3D) -> Frame3D:
        return Frame3D(
            name=frame.name,
            origin=self.transform_point(frame.origin),
            axis_x=self.transform_direction(frame.axis_x).normalized(),
            axis_y=self.transform_direction(frame.axis_y).normalized(),
            axis_z=self.transform_direction(frame.axis_z).normalized(),
            ambiguous=frame.ambiguous,
            synthesized=frame.synthesized,
        )
