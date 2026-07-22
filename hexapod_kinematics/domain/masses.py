"""Load and validate masses_996.yml; mass-weighted COM helpers."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from hexapod_kinematics.domain.frames_world import MountWorld


class MassesConfigError(ValueError):
    """Invalid masses configuration."""


@dataclass(frozen=True, slots=True)
class MassBreakdown:
    links_g: float
    servos_g: float
    body_g: float
    battery_misc_g: float
    total_g: float
    expected_total_g: float
    warnings: tuple[str, ...]

    def as_dict(self) -> dict[str, Any]:
        return {
            "links_g": self.links_g,
            "servos_g": self.servos_g,
            "body_g": self.body_g,
            "battery_misc_g": self.battery_misc_g,
            "total_g": self.total_g,
            "expected_total_g": self.expected_total_g,
            "warnings": list(self.warnings),
        }


def load_masses_config(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise MassesConfigError(f"masses config not found: {path}")
    with path.open(encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise MassesConfigError(f"masses config must be a mapping: {path}")
    for key in ("total_robot_g", "links_g", "servos_g", "body_g"):
        if key not in data:
            raise MassesConfigError(f"masses config missing {key}")
    return data


def validate_masses(data: dict[str, Any]) -> MassBreakdown:
    links = data["links_g"]
    links_total = 6.0 * (
        float(links["coxa"]) + float(links["femur"]) + float(links["tibia"])
    )
    servos = data["servos_g"]
    servos_total = float(servos["unit"]) * float(servos["count"])
    body_total = 0.0
    for part in data["body_g"].get("parts", []):
        body_total += float(part["mass_g"]) * float(part.get("count", 1))
    battery = float(data.get("battery_misc_g", 0.0))
    total = links_total + servos_total + body_total + battery
    expected = float(data["total_robot_g"])
    warnings: list[str] = []
    if abs(total - expected) > 1.0:
        warnings.append(
            f"mass sum {total:.1f} g != total_robot_g {expected:.1f} g"
        )
    return MassBreakdown(
        links_g=links_total,
        servos_g=servos_total,
        body_g=body_total,
        battery_misc_g=battery,
        total_g=total,
        expected_total_g=expected,
        warnings=tuple(warnings),
    )


def body_battery_mass_g(masses: dict[str, Any]) -> float:
    body_total = 0.0
    for part in masses["body_g"].get("parts", []):
        body_total += float(part["mass_g"]) * float(part.get("count", 1))
    return body_total + float(masses.get("battery_misc_g", 0.0))


def mass_weighted_com_xy(
    *,
    mounts: list[MountWorld],
    chains: dict[int, np.ndarray] | None,
    masses: dict[str, Any],
) -> tuple[np.ndarray, str]:
    """
    Mass-weighted COM in XY.

    Body + battery at geometric mean of mounts (+ optional com_offset).
    Link masses at mid-segment of each chain when provided; otherwise at mounts.
    Servos: 1 per joint at mid-coxa / mid-femur / mid-tibia when chain exists.
    """
    origins = np.array([m.origin for m in mounts], dtype=float)
    body_xy = origins.mean(axis=0)[:2]
    offset = np.asarray(
        masses.get("com_offset_mm", [0.0, 0.0, 0.0]), dtype=float
    ).reshape(3)[:2]

    links = masses["links_g"]
    m_coxa = float(links["coxa"])
    m_femur = float(links["femur"])
    m_tibia = float(links["tibia"])
    m_servo = float(masses["servos_g"]["unit"])
    frac = float(masses.get("com_link_fraction", 0.5))
    body_mass = body_battery_mass_g(masses)

    mass_sum = body_mass
    com = (body_xy + offset) * body_mass

    if chains:
        for _lid, chain in chains.items():
            pts = np.asarray(chain, dtype=float)
            if pts.shape[0] < 4:
                continue
            mid_coxa = pts[0] + frac * (pts[1] - pts[0])
            mid_femur = pts[1] + frac * (pts[2] - pts[1])
            mid_tibia = pts[2] + frac * (pts[3] - pts[2])
            for mid, m_link in (
                (mid_coxa, m_coxa + m_servo),
                (mid_femur, m_femur + m_servo),
                (mid_tibia, m_tibia + m_servo),
            ):
                mass_sum += m_link
                com = com + mid[:2] * m_link
    else:
        per_leg = m_coxa + m_femur + m_tibia + 3.0 * m_servo
        for mnt in mounts:
            mass_sum += per_leg
            com = com + mnt.origin[:2] * per_leg

    if mass_sum <= 0:
        return body_xy + offset, "geometric_fallback"
    return com / mass_sum, "mass_model"
