"""Interactive matplotlib slider over precomputed animation frames (no re-sim)."""

from __future__ import annotations

from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider

from hexapod_kinematics.presentation.animate_motion import (
    _collect_bounds,
    _draw_frame,
    build_footfall,
)


def interactive_motion(
    anim_frames: list[dict[str, Any]],
    mounts_xy: np.ndarray,
    *,
    title: str = "Hexapod motion",
) -> None:
    """
    Show 3-panel viewer; slider only indexes into cached frames.
    FK/IK must already be baked into anim_frames['chains'].
    """
    if not anim_frames:
        raise ValueError("No animation frames")

    frames: list[dict[str, Any]] = []
    for fr in anim_frames:
        f = dict(fr)
        f["foot_xy"] = {int(k): v for k, v in (fr.get("foot_xy") or {}).items()}
        f["roles"] = {int(k): v for k, v in (fr.get("roles") or {}).items()}
        f["chains"] = {int(k): v for k, v in (fr.get("chains") or {}).items()}
        frames.append(f)

    footfall = build_footfall(frames)
    xlim, ylim, zlim = _collect_bounds(frames, mounts_xy)
    fig = plt.figure(figsize=(13, 9))
    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    ax_top = fig.add_subplot(2, 2, 2)
    ax_foot = fig.add_subplot(2, 1, 2)
    plt.subplots_adjust(bottom=0.12)

    def redraw(i: int) -> None:
        idx = int(np.clip(i, 0, len(frames) - 1))
        fr = frames[idx]
        t_norm = (idx % footfall.shape[1]) / max(footfall.shape[1], 1)
        _draw_frame(
            ax3d,
            ax_top,
            ax_foot,
            fr,
            mounts_xy,
            footfall,
            t_norm,
            title,
            xlim=xlim,
            ylim=ylim,
            zlim=zlim,
        )
        fig.canvas.draw_idle()

    ax_slider = fig.add_axes((0.15, 0.02, 0.7, 0.03))
    slider = Slider(
        ax_slider,
        "frame",
        0,
        len(frames) - 1,
        valinit=0,
        valstep=1,
    )
    slider.on_changed(lambda v: redraw(v))
    redraw(0)
    plt.show()
