"""Link length computation and Z-axis alignment checks."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from hexapod_kinematics.domain.frames import Frame3D, Vec3


@dataclass
class LinkLengthResult:
    length_mm: float
    axis_alignment_ok: bool
    angle_deg: float
    lateral_mm: float
    synthesized_out: bool
    ambiguous: bool
    warnings: list[str] = field(default_factory=list)
    frame_in: Frame3D | None = None
    frame_out: Frame3D | None = None
    missing: list[str] = field(default_factory=list)


def synthesize_out(
    frame_in: Frame3D,
    axis_length_mm: float,
    name: str = "LCS_out_synth",
) -> Frame3D:
    origin = frame_in.origin + frame_in.axis_z.normalized() * axis_length_mm
    return Frame3D(
        name=name,
        origin=origin,
        axis_x=frame_in.axis_x,
        axis_y=frame_in.axis_y,
        axis_z=frame_in.axis_z,
        ambiguous=frame_in.ambiguous,
        synthesized=True,
    )


def _angle_deg(a: Vec3, b: Vec3) -> float:
    na = a.normalized()
    nb = b.normalized()
    cos_a = max(-1.0, min(1.0, na.dot(nb)))
    return math.degrees(math.acos(abs(cos_a)))


def compute_link_length(
    frame_in: Frame3D | None,
    frame_out: Frame3D | None,
    *,
    angle_tol_deg: float,
    lateral_tol_mm: float,
    synthesize_length_mm: float | None = None,
) -> LinkLengthResult:
    warnings: list[str] = []
    missing: list[str] = []

    if frame_in is None:
        missing.append("in")
    if frame_out is None and synthesize_length_mm is None:
        missing.append("out")

    if frame_in is None:
        return LinkLengthResult(
            length_mm=0.0,
            axis_alignment_ok=False,
            angle_deg=0.0,
            lateral_mm=0.0,
            synthesized_out=False,
            ambiguous=False,
            warnings=warnings + ["missing LCS role 'in'"],
            missing=missing,
        )

    synthesized = False
    out = frame_out
    if out is None:
        assert synthesize_length_mm is not None
        out = synthesize_out(frame_in, synthesize_length_mm)
        synthesized = True
        warnings.append(
            f"synthesized LCS_out along Z using {synthesize_length_mm} mm"
        )

    delta = out.origin - frame_in.origin
    length = delta.norm()
    z_axis = frame_in.axis_z.normalized()
    angle = _angle_deg(delta, z_axis) if length > 0 else 0.0
    # lateral distance from out origin to the Z line through in.origin
    proj = z_axis * delta.dot(z_axis)
    lateral = (delta - proj).norm()
    alignment_ok = angle <= angle_tol_deg and lateral <= lateral_tol_mm
    if not alignment_ok:
        warnings.append(
            f"origin_out-origin_in not aligned with Z: "
            f"angle_deg={angle:.4f} (tol={angle_tol_deg}), "
            f"lateral_mm={lateral:.4f} (tol={lateral_tol_mm})"
        )

    ambiguous = bool(frame_in.ambiguous or out.ambiguous)
    if ambiguous:
        warnings.append("one or more LCS marked ambiguous (duplicate names)")

    return LinkLengthResult(
        length_mm=length,
        axis_alignment_ok=alignment_ok,
        angle_deg=angle,
        lateral_mm=lateral,
        synthesized_out=synthesized,
        ambiguous=ambiguous,
        warnings=warnings,
        frame_in=frame_in,
        frame_out=out,
        missing=missing,
    )
