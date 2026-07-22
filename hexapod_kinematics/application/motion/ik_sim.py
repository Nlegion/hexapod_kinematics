"""IK gait simulation."""

from __future__ import annotations

from typing import Any

import numpy as np

from hexapod_kinematics.core.constants.world_frame import LEG_SHORT_NAMES
from hexapod_kinematics.domain.body_frame_kin import chain_to_body
from hexapod_kinematics.domain.frames_world import MountWorld
from hexapod_kinematics.domain.gait_ik import sample_ik_frame
from hexapod_kinematics.domain.gait_metrics import band_of, compute_frame_metrics
from hexapod_kinematics.domain.gait_pulse import Direction
from hexapod_kinematics.domain.kinematics import (
    JointAngles,
    LinkLengths,
    angles_to_pulses,
    forward_kinematics,
)


def simulate_ik(
    *,
    gait: dict[str, Any],
    mounts: list[MountWorld],
    lengths: LinkLengths,
    cycles: int,
    direction: Direction,
    body_height_mm: float,
    frames_per_cycle: int = 40,
    masses: dict[str, Any] | None = None,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    group1 = set(int(x) for x in gait["tripod_group_1"])
    com_off = np.asarray(gait.get("com_offset_mm", [0, 0, 0]), dtype=float)
    stride = float(gait.get("stride_mm", 40.0))
    n_pulse = len(gait["transfer_traj"])
    delay = float(gait.get("step_delay_ms", 150))
    cycle_time_ms = 2 * n_pulse * delay
    dt = cycle_time_ms / frames_per_cycle
    body_speed = stride / (cycle_time_ms / 1000.0)

    leg_rows: list[dict[str, Any]] = []
    summary_rows: list[dict[str, Any]] = []
    anim: list[dict[str, Any]] = []
    last_angles: dict[int, JointAngles] = {}
    fail_hist: list[bool] = []

    total_frames = cycles * frames_per_cycle
    for frame_idx in range(total_frames):
        cycle = frame_idx // frames_per_cycle
        s_global = (frame_idx % frames_per_cycle) / frames_per_cycle
        t_ms = frame_idx * dt
        samples = sample_ik_frame(
            s_global=s_global,
            mounts=mounts,
            lengths=lengths,
            gait=gait,
            body_height_mm=body_height_mm,
            last_angles=last_angles,
        )

        foot_xy: dict[int, np.ndarray] = {}
        chains: dict[int, np.ndarray] = {}
        phases: dict[int, float] = {}
        stance_ids: list[int] = []
        roles: dict[int, str] = {}
        frame_flags: list[bool] = []

        for sample in samples:
            lid = sample.leg_id
            mount = mounts[lid]
            last_angles[lid] = sample.angles
            tip = forward_kinematics(sample.angles, lengths)
            foot_body = mount.origin + mount.rotation @ np.array(
                [tip.x, tip.y, tip.z], dtype=float
            )
            if sample.ik_ok:
                foot_body = sample.target_body
                tip = sample.target_coxa
            chain = chain_to_body(sample.angles, lengths, mount)
            if sample.ik_ok:
                chain[-1] = foot_body
            foot_xy[lid] = foot_body[:2]
            chains[lid] = chain
            phases[lid] = sample.phase_s
            roles[lid] = sample.role
            if sample.role == "support":
                stance_ids.append(lid)
            frame_flags.append(sample.ik_ok)
            fail_hist.append(sample.ik_ok)
            pulses = angles_to_pulses(
                sample.angles,
                neutral=float(gait.get("neutral", 1500)),
                deg_per_us=float(gait.get("deg_per_us", 0.18)),
            )
            reach = float(
                np.linalg.norm(
                    [sample.target_coxa.x, sample.target_coxa.y, sample.target_coxa.z]
                )
            )
            leg_rows.append(
                {
                    "frame_idx": frame_idx,
                    "t_ms": t_ms,
                    "mode": "ik",
                    "metrics_basis": "ik",
                    "cycle": cycle,
                    "phase": 1 if s_global < 0.5 else 2,
                    "step": int((s_global % 0.5) * 2 * n_pulse) % n_pulse,
                    "direction": direction,
                    "leg_id": lid,
                    "leg_name": LEG_SHORT_NAMES[lid],
                    "band": band_of(lid),
                    "role": sample.role,
                    "pulse_c": pulses.coxa,
                    "pulse_f": pulses.femur,
                    "pulse_t": pulses.tibia,
                    "angle_c_rad": sample.angles.coxa,
                    "angle_f_rad": sample.angles.femur,
                    "angle_t_rad": sample.angles.tibia,
                    "foot_body_x": foot_body[0],
                    "foot_body_y": foot_body[1],
                    "foot_body_z": foot_body[2],
                    "foot_coxa_x": tip.x,
                    "foot_coxa_y": tip.y,
                    "foot_coxa_z": tip.z,
                    "ik_ok": sample.ik_ok,
                    "reach_mm": reach,
                    "stride_mm": stride,
                }
            )

        window = fail_hist[-(6 * frames_per_cycle) :]
        fail_ratio = sum(1 for ok in window if not ok) / max(len(window), 1)

        metrics = compute_frame_metrics(
            mounts=mounts,
            foot_body_xy=foot_xy,
            stance_ids=stance_ids,
            com_offset_mm=com_off,
            group1=group1,
            phases=phases,
            ik_flags=frame_flags,
            body_speed_est_mm_s=body_speed,
            duty_instant=len(stance_ids) / 6.0,
            chains=chains,
            masses=masses,
        )
        summary_rows.append(
            {
                "frame_idx": frame_idx,
                "t_ms": t_ms,
                "mode": "ik",
                "metrics_basis": "ik",
                "cycle": cycle,
                "phase": 1 if s_global < 0.5 else 2,
                "step": int((s_global % 0.5) * 2 * n_pulse) % n_pulse,
                "com_x": metrics.com_xy[0],
                "com_y": metrics.com_xy[1],
                "com_source": metrics.com_source,
                "n_stance": metrics.n_stance,
                "support_ok": metrics.support_ok,
                "support_margin_mm": metrics.support_margin_mm,
                "duty_mean": metrics.duty_mean,
                "phase_err_mean_deg": metrics.phase_err_mean_deg,
                "phase_err_max_deg": metrics.phase_err_max_deg,
                "ik_fail_ratio": fail_ratio,
                "body_speed_est_mm_s": body_speed,
                "hip_z": float(mounts[0].origin[2]) if mounts else body_height_mm,
            }
        )
        anim.append(
            {
                "frame_idx": frame_idx,
                "t_ms": t_ms,
                "mode": "ik",
                "metrics_basis": "ik",
                "chains": {k: v.tolist() for k, v in chains.items()},
                "roles": roles,
                "com_xy": metrics.com_xy.tolist(),
                "com_source": metrics.com_source,
                "stance_ids": stance_ids,
                "foot_xy": {k: v.tolist() for k, v in foot_xy.items()},
                "support_ok": metrics.support_ok,
            }
        )

    return leg_rows, summary_rows, anim
