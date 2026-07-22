"""2–3 DoF leg dynamics (Lagrange II) using masses_996 mid-link CoM model."""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from hexapod_kinematics.domain.kinematics import JointAngles, LinkLengths

G = 9.80665  # m/s^2


def _mid_inertia_point_mass(mass_kg: float, length_m: float, fraction: float = 0.5) -> float:
    """Point-mass inertia about proximal joint: m*(f*L)^2."""
    return mass_kg * (fraction * length_m) ** 2


def torques_planar_femur_tibia(
    *,
    angles: JointAngles,
    omega: tuple[float, float, float],
    alpha: tuple[float, float, float],
    lengths: LinkLengths,
    masses_g: dict[str, float],
    servo_g: float = 55.0,
    com_fraction: float = 0.5,
) -> dict[str, float]:
    """
    Simplified Lagrange-ish torques for femur–tibia plane (coxa optional).

    Units: lengths mm→m, masses g→kg, angles rad, omega/alpha rad/s(/s^2).
    Servo masses as point masses at joints (hip/knee/ankle approx).
    Gravity only + inertial diagonal terms (no Coriolis cross for v1).
    """
    Lf = lengths.femur / 1000.0
    Lt = lengths.tibia / 1000.0
    Lc = lengths.coxa / 1000.0

    m_f = (float(masses_g["femur"]) + float(servo_g)) / 1000.0
    m_t = (float(masses_g["tibia"]) + float(servo_g)) / 1000.0
    m_c = (float(masses_g["coxa"]) + float(servo_g)) / 1000.0

    # CoM distances from proximal joint along link
    r_f = com_fraction * Lf
    r_t = com_fraction * Lt

    qf = float(angles.femur)
    qt = float(angles.tibia)
    _wc, wf, wt = omega
    _ac, af, at = alpha

    # Gravity: potential uses vertical component sin(joint) with z-up FK convention
    # M_g ≈ m g r cos(q) for each link (planar approx)
    M_femur_g = m_f * G * r_f * np.cos(qf) + m_t * G * (
        Lf * np.cos(qf) + r_t * np.cos(qf + qt)
    )
    M_tibia_g = m_t * G * r_t * np.cos(qf + qt)
    M_coxa_g = m_c * G * (com_fraction * Lc) * 0.0  # coxa horizontal yaw: no gravity torque

    I_f = _mid_inertia_point_mass(m_f, Lf, com_fraction) + m_t * Lf**2
    I_t = _mid_inertia_point_mass(m_t, Lt, com_fraction)

    M_femur = float(I_f * af + M_femur_g)
    M_tibia = float(I_t * at + M_tibia_g)
    M_coxa = float(_mid_inertia_point_mass(m_c, Lc, com_fraction) * _ac + M_coxa_g)

    return {
        "M_coxa_nm": M_coxa,
        "M_femur_nm": M_femur,
        "M_tibia_nm": M_tibia,
    }


def estimate_rates_from_log(
    rows: list[dict[str, Any]],
    leg_id: int,
) -> list[dict[str, Any]]:
    """Finite-difference ω, α from one leg's angle_* / t_ms rows (sorted)."""
    seq = sorted(
        (r for r in rows if int(r["leg_id"]) == leg_id),
        key=lambda r: (float(r["t_ms"]), int(r["frame_idx"])),
    )
    out: list[dict[str, Any]] = []
    for i, r in enumerate(seq):
        angs = np.array(
            [float(r["angle_c_rad"]), float(r["angle_f_rad"]), float(r["angle_t_rad"])]
        )
        if i == 0 or i == len(seq) - 1:
            omega = np.zeros(3)
            alpha = np.zeros(3)
        else:
            t0 = float(seq[i - 1]["t_ms"]) / 1000.0
            t1 = float(seq[i + 1]["t_ms"]) / 1000.0
            a0 = np.array(
                [
                    float(seq[i - 1]["angle_c_rad"]),
                    float(seq[i - 1]["angle_f_rad"]),
                    float(seq[i - 1]["angle_t_rad"]),
                ]
            )
            a1 = np.array(
                [
                    float(seq[i + 1]["angle_c_rad"]),
                    float(seq[i + 1]["angle_f_rad"]),
                    float(seq[i + 1]["angle_t_rad"]),
                ]
            )
            dt = max(t1 - t0, 1e-6)
            omega = (a1 - a0) / dt
            # central second derivative
            alpha = (a1 - 2 * angs + a0) / ((dt / 2) ** 2 + 1e-12)
        out.append(
            {
                "frame_idx": r["frame_idx"],
                "t_ms": r["t_ms"],
                "leg_id": leg_id,
                "angles": JointAngles(float(angs[0]), float(angs[1]), float(angs[2])),
                "omega": (float(omega[0]), float(omega[1]), float(omega[2])),
                "alpha": (float(alpha[0]), float(alpha[1]), float(alpha[2])),
                "role": r.get("role"),
            }
        )
    return out


def compute_torques_for_log(
    legs: list[dict[str, Any]],
    *,
    lengths: LinkLengths,
    masses_cfg: dict[str, Any],
    leg_ids: Iterable[int] | None = None,
) -> list[dict[str, Any]]:
    links = masses_cfg["links_g"]
    servo = float(masses_cfg["servos_g"]["unit"])
    frac = float(masses_cfg.get("com_link_fraction", 0.5))
    ids = list(leg_ids) if leg_ids is not None else list(range(6))
    rows_out: list[dict[str, Any]] = []
    for lid in ids:
        samples = estimate_rates_from_log(legs, lid)
        for s in samples:
            # Dynamics meaningful mainly on support; still compute all
            tq = torques_planar_femur_tibia(
                angles=s["angles"],
                omega=s["omega"],
                alpha=s["alpha"],
                lengths=lengths,
                masses_g=links,
                servo_g=servo,
                com_fraction=frac,
            )
            rows_out.append(
                {
                    "frame_idx": s["frame_idx"],
                    "t_ms": s["t_ms"],
                    "leg_id": lid,
                    "role": s["role"],
                    **tq,
                }
            )
    return rows_out


def write_torques_csv(path: Path, rows: list[dict[str, Any]]) -> Path:
    fields = [
        "frame_idx",
        "t_ms",
        "leg_id",
        "role",
        "M_coxa_nm",
        "M_femur_nm",
        "M_tibia_nm",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for row in rows:
            w.writerow(row)
    return path
