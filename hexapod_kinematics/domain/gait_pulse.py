"""Pulse-based tripod gait replay (firmware GaitService.applyTrajectory)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

from hexapod_kinematics.domain.kinematics import ServoPulses

Direction = Literal["forward", "backward", "turn_left", "turn_right"]


@dataclass(frozen=True, slots=True)
class PulseStep:
    phase: int  # 1 or 2
    step: int  # 0..traj_steps-1
    cycle: int
    t_ms: float
    pulses: dict[int, ServoPulses]  # leg_id → pulses
    roles: dict[int, str]  # transfer | support


def _coxa_direction(
    leg_id: int,
    base_dirs: list[int],
    direction: Direction,
) -> int:
    d = int(base_dirs[leg_id])
    is_left = leg_id in (3, 4, 5)  # RL, ML, FL
    if direction == "backward":
        d *= -1
    elif direction == "turn_left" and is_left:
        d *= -1
    elif direction == "turn_right" and not is_left:
        d *= -1
    return d


def apply_trajectory_pulses(
    traj_row: list[int],
    *,
    leg_id: int,
    gait: dict[str, Any],
    direction: Direction,
    coxa_scale: float = 1.0,
) -> ServoPulses:
    neutral = int(gait.get("neutral", 1500))
    min_p = int(gait.get("min_pulse", 1000))
    max_p = int(gait.get("max_pulse", 2000))
    dirs = list(gait.get("leg_forward_directions", [1] * 6))

    coxa_pulse, femur_pulse, tibia_pulse = (int(traj_row[0]), int(traj_row[1]), int(traj_row[2]))
    coxa_dir = _coxa_direction(leg_id, dirs, direction)
    coxa_offset = coxa_pulse - neutral
    coxa_pulse = int(round(neutral + coxa_offset * coxa_dir * coxa_scale))
    coxa_pulse = max(min_p, min(max_p, coxa_pulse))
    femur_pulse = max(min_p, min(max_p, femur_pulse))
    tibia_pulse = max(min_p, min(max_p, tibia_pulse))
    return ServoPulses(coxa_pulse, femur_pulse, tibia_pulse)


def iter_pulse_gait(
    gait: dict[str, Any],
    *,
    cycles: int = 1,
    direction: Direction = "forward",
    coxa_scale: float = 1.0,
) -> list[PulseStep]:
    transfer = gait["transfer_traj"]
    support = gait["support_traj"]
    g1 = set(int(x) for x in gait["tripod_group_1"])
    g2 = set(int(x) for x in gait["tripod_group_2"])
    n_steps = len(transfer)
    delay = float(gait.get("step_delay_ms", 150))
    frames: list[PulseStep] = []
    t = 0.0

    for cycle in range(cycles):
        for phase in (1, 2):
            transfer_legs = g1 if phase == 1 else g2
            for step in range(n_steps):
                pulses: dict[int, ServoPulses] = {}
                roles: dict[int, str] = {}
                for leg_id in range(6):
                    if leg_id in transfer_legs:
                        row = transfer[step]
                        roles[leg_id] = "transfer"
                    else:
                        row = support[step]
                        roles[leg_id] = "support"
                    pulses[leg_id] = apply_trajectory_pulses(
                        row,
                        leg_id=leg_id,
                        gait=gait,
                        direction=direction,
                        coxa_scale=coxa_scale,
                    )
                frames.append(
                    PulseStep(
                        phase=phase,
                        step=step,
                        cycle=cycle,
                        t_ms=t,
                        pulses=pulses,
                        roles=roles,
                    )
                )
                t += delay
    return frames


def estimate_coxa_scale_for_stride(
    *,
    current_stride_mm: float,
    target_stride_mm: float,
) -> float:
    if abs(current_stride_mm) < 1e-6:
        return 1.0
    return float(target_stride_mm / current_stride_mm)
