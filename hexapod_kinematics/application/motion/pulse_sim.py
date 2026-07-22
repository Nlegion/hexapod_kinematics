"""Pulse gait simulation: pulse_raw and pulse_aligned (no IK retarget)."""

from __future__ import annotations

from typing import Any, Literal

import numpy as np

from hexapod_kinematics.core.constants.world_frame import LEG_SHORT_NAMES
from hexapod_kinematics.domain.body_frame_kin import chain_to_body
from hexapod_kinematics.domain.frames_world import MountWorld
from hexapod_kinematics.domain.gait_metrics import band_of, compute_frame_metrics
from hexapod_kinematics.domain.gait_pulse import Direction, iter_pulse_gait
from hexapod_kinematics.domain.kinematics import (
    JointAngles,
    LinkLengths,
    forward_kinematics,
    pulses_to_angles,
)
from hexapod_kinematics.domain.neutral_pose import resolve_servo_neutral


PulseMode = Literal["pulse_raw", "pulse_aligned"]


def measure_pulse_stride_mm(frames_data: list[dict[str, Any]]) -> float:
    """Mean |Δfoot_x| over support phase per leg across first cycle."""
    by_leg: dict[int, list[float]] = {i: [] for i in range(6)}
    for row in frames_data:
        if row.get("cycle", 0) != 0:
            continue
        if row.get("role") != "support":
            continue
        by_leg[int(row["leg_id"])].append(float(row["foot_body_x"]))
    spans = []
    for xs in by_leg.values():
        if len(xs) >= 2:
            spans.append(abs(max(xs) - min(xs)))
    return float(np.mean(spans)) if spans else 0.0


def _shift_mounts_z(mounts: list[MountWorld], dz: float) -> list[MountWorld]:
    out: list[MountWorld] = []
    for m in mounts:
        origin = np.asarray(m.origin, dtype=float).copy()
        origin[2] += dz
        out.append(
            MountWorld(
                leg_id=m.leg_id,
                leg_name=m.leg_name,
                origin=origin,
                axis_x=np.asarray(m.axis_x, dtype=float).copy(),
                axis_y=np.asarray(m.axis_y, dtype=float).copy(),
                axis_z=np.asarray(m.axis_z, dtype=float).copy(),
                yaw_deg=m.yaw_deg,
            )
        )
    return out


def _fk_frame(
    *,
    mounts: list[MountWorld],
    lengths: LinkLengths,
    step: Any,
    neutral: float,
    deg_per_us: float,
    neutral_angles: JointAngles,
) -> tuple[
    dict[int, JointAngles],
    dict[int, Any],
    dict[int, Any],
    dict[int, np.ndarray],
    dict[int, np.ndarray],
]:
    angles_map: dict[int, JointAngles] = {}
    pulses_map: dict[int, Any] = {}
    tips_coxa: dict[int, Any] = {}
    chains: dict[int, np.ndarray] = {}
    foot_body: dict[int, np.ndarray] = {}
    for mount in mounts:
        lid = mount.leg_id
        pulses = step.pulses[lid]
        angles = pulses_to_angles(
            pulses,
            neutral=neutral,
            deg_per_us=deg_per_us,
            neutral_angles=neutral_angles,
        )
        tip = forward_kinematics(angles, lengths)
        chain = chain_to_body(angles, lengths, mount)
        angles_map[lid] = angles
        pulses_map[lid] = pulses
        tips_coxa[lid] = tip
        chains[lid] = chain
        foot_body[lid] = chain[-1].copy()
    return angles_map, pulses_map, tips_coxa, chains, foot_body


def simulate_pulse(
    *,
    gait: dict[str, Any],
    mounts: list[MountWorld],
    lengths: LinkLengths,
    cycles: int,
    direction: Direction,
    coxa_scale: float,
    pulse_mode: PulseMode,
    masses: dict[str, Any] | None = None,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    """
    Returns leg_rows, summary_rows, anim_frames.

    pulse_raw: FK as placed (after place_hips_above_ground).
    pulse_aligned: same angles; rigid ΔZ so min(stance tip_z) → 0 each frame.
    Never uses IK to retarget feet.
    """
    neutral = float(gait.get("neutral", 1500))
    deg_per_us = float(gait.get("deg_per_us", 0.18))
    neutral_angles = resolve_servo_neutral(gait, lengths)
    group1 = set(int(x) for x in gait["tripod_group_1"])
    com_off = np.asarray(gait.get("com_offset_mm", [0, 0, 0]), dtype=float)
    delay = float(gait.get("step_delay_ms", 150))
    steps = iter_pulse_gait(
        gait, cycles=cycles, direction=direction, coxa_scale=coxa_scale
    )
    metrics_basis = pulse_mode

    leg_rows: list[dict[str, Any]] = []
    summary_rows: list[dict[str, Any]] = []
    anim: list[dict[str, Any]] = []
    prev_com = None

    for frame_idx, step in enumerate(steps):
        angles_map, pulses_map, tips_coxa, chains, foot_body = _fk_frame(
            mounts=mounts,
            lengths=lengths,
            step=step,
            neutral=neutral,
            deg_per_us=deg_per_us,
            neutral_angles=neutral_angles,
        )
        # Raw feet before alignment (always logged)
        foot_raw = {lid: foot_body[lid].copy() for lid in range(6)}
        chains_raw = {lid: chains[lid].copy() for lid in range(6)}

        stance_ids = [lid for lid in range(6) if step.roles[lid] == "support"]
        aligned_mounts = mounts
        dz = 0.0
        if pulse_mode == "pulse_aligned" and stance_ids:
            min_tip_z = min(float(foot_body[i][2]) for i in stance_ids)
            dz = -min_tip_z
            aligned_mounts = _shift_mounts_z(mounts, dz)
            angles_map, pulses_map, tips_coxa, chains, foot_body = _fk_frame(
                mounts=aligned_mounts,
                lengths=lengths,
                step=step,
                neutral=neutral,
                deg_per_us=deg_per_us,
                neutral_angles=neutral_angles,
            )

        phases: dict[int, float] = {}
        n = len(gait["transfer_traj"])
        for lid in range(6):
            local_step = step.step + (0 if step.phase == 1 else n)
            phase_s = (local_step / (2 * n)) % 1.0
            if lid not in group1:
                phase_s = (phase_s + 0.5) % 1.0
            phases[lid] = phase_s

        foot_xy = {lid: foot_body[lid][:2] for lid in range(6)}
        frame_ik_flags = [True] * 6  # pulse has no IK

        for lid in range(6):
            tip = tips_coxa[lid]
            angles = angles_map[lid]
            pulses = pulses_map[lid]
            fb = foot_body[lid]
            fr = foot_raw[lid]
            reach = float(np.linalg.norm([tip.x, tip.y, tip.z]))
            leg_rows.append(
                {
                    "frame_idx": frame_idx,
                    "t_ms": step.t_ms,
                    "mode": pulse_mode,
                    "metrics_basis": metrics_basis,
                    "cycle": step.cycle,
                    "phase": step.phase,
                    "step": step.step,
                    "direction": direction,
                    "leg_id": lid,
                    "leg_name": LEG_SHORT_NAMES[lid],
                    "band": band_of(lid),
                    "role": step.roles[lid],
                    "pulse_c": pulses.coxa,
                    "pulse_f": pulses.femur,
                    "pulse_t": pulses.tibia,
                    "angle_c_rad": angles.coxa,
                    "angle_f_rad": angles.femur,
                    "angle_t_rad": angles.tibia,
                    "foot_body_x": fb[0],
                    "foot_body_y": fb[1],
                    "foot_body_z": fb[2],
                    "foot_raw_x": fr[0],
                    "foot_raw_y": fr[1],
                    "foot_raw_z": fr[2],
                    "foot_coxa_x": tip.x,
                    "foot_coxa_y": tip.y,
                    "foot_coxa_z": tip.z,
                    "align_dz_mm": dz,
                    "ik_ok": True,
                    "reach_mm": reach,
                    "stride_mm": "",
                }
            )

        speed = 0.0
        metrics = compute_frame_metrics(
            mounts=aligned_mounts,
            foot_body_xy=foot_xy,
            stance_ids=stance_ids,
            com_offset_mm=com_off,
            group1=group1,
            phases=phases,
            ik_flags=frame_ik_flags,
            body_speed_est_mm_s=speed,
            duty_instant=len(stance_ids) / 6.0,
            chains=chains,
            masses=masses,
        )
        if prev_com is not None and delay > 0:
            speed = float(np.linalg.norm(metrics.com_xy - prev_com) / (delay / 1000.0))
            metrics = compute_frame_metrics(
                mounts=aligned_mounts,
                foot_body_xy=foot_xy,
                stance_ids=stance_ids,
                com_offset_mm=com_off,
                group1=group1,
                phases=phases,
                ik_flags=frame_ik_flags,
                body_speed_est_mm_s=speed,
                duty_instant=len(stance_ids) / 6.0,
                chains=chains,
                masses=masses,
            )
        prev_com = metrics.com_xy.copy()

        summary_rows.append(
            {
                "frame_idx": frame_idx,
                "t_ms": step.t_ms,
                "mode": pulse_mode,
                "metrics_basis": metrics_basis,
                "cycle": step.cycle,
                "phase": step.phase,
                "step": step.step,
                "com_x": metrics.com_xy[0],
                "com_y": metrics.com_xy[1],
                "com_source": metrics.com_source,
                "n_stance": metrics.n_stance,
                "support_ok": metrics.support_ok,
                "support_margin_mm": metrics.support_margin_mm,
                "duty_mean": metrics.duty_mean,
                "phase_err_mean_deg": metrics.phase_err_mean_deg,
                "phase_err_max_deg": metrics.phase_err_max_deg,
                "ik_fail_ratio": metrics.ik_fail_ratio,
                "body_speed_est_mm_s": metrics.body_speed_est_mm_s,
                "align_dz_mm": dz,
                "hip_z": float(aligned_mounts[0].origin[2]) if aligned_mounts else 0.0,
            }
        )
        anim.append(
            {
                "frame_idx": frame_idx,
                "t_ms": step.t_ms,
                "mode": pulse_mode,
                "metrics_basis": metrics_basis,
                "chains": {k: v.tolist() for k, v in chains.items()},
                "chains_raw": {k: v.tolist() for k, v in chains_raw.items()},
                "roles": dict(step.roles),
                "com_xy": metrics.com_xy.tolist(),
                "com_source": metrics.com_source,
                "stance_ids": stance_ids,
                "foot_xy": {k: v.tolist() for k, v in foot_xy.items()},
                "support_ok": metrics.support_ok,
                "align_dz_mm": dz,
            }
        )

    return leg_rows, summary_rows, anim
