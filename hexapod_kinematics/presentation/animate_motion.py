"""Animate hexapod motion: 3D stick + top support/COM + Hildebrand footfall."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Polygon
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from hexapod_kinematics.core.constants.world_frame import LEG_SHORT_NAMES

COLORS = ("#e74c3c", "#e67e22", "#f1c40f", "#2ecc71", "#3498db", "#9b59b6")


def _draw_frame(
    ax3d,
    ax_top,
    ax_foot,
    frame: dict[str, Any],
    mounts_xy: np.ndarray,
    footfall: np.ndarray,
    t_norm: float,
    title: str,
) -> None:
    ax3d.cla()
    ax_top.cla()
    ax_foot.cla()

    ax3d.set_title(title)
    ax3d.set_xlabel("X fwd")
    ax3d.set_ylabel("Y left")
    ax3d.set_zlabel("Z up")

    chains = frame["chains"]
    all_pts = []
    for lid_s, pts in chains.items():
        lid = int(lid_s)
        arr = np.asarray(pts, dtype=float)
        all_pts.append(arr)
        ax3d.plot(arr[:, 0], arr[:, 1], arr[:, 2], "-o", color=COLORS[lid], ms=3)
    # Body polygon at hip height
    if len(mounts_xy):
        order = np.argsort(np.arctan2(mounts_xy[:, 1], mounts_xy[:, 0]))
        hx = mounts_xy[order, 0]
        hy = mounts_xy[order, 1]
        hz = np.zeros_like(hx)
        if all_pts:
            hz[:] = float(np.mean([p[0, 2] for p in all_pts]))
        ax3d.plot(
            np.append(hx, hx[0]),
            np.append(hy, hy[0]),
            np.append(hz, hz[0]),
            color="#7f8c8d",
            lw=1.5,
        )

    # Ground plane z=0
    if all_pts:
        cloud = np.vstack(all_pts)
        pad = 40.0
        x0, x1 = float(cloud[:, 0].min() - pad), float(cloud[:, 0].max() + pad)
        y0, y1 = float(cloud[:, 1].min() - pad), float(cloud[:, 1].max() + pad)
        xx, yy = np.meshgrid([x0, x1], [y0, y1])
        ax3d.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.15, color="#95a5a6")
        span = np.ptp(cloud, axis=0)
        span = np.maximum(span, 1.0)
        # Keep vertical scale readable vs floor gap
        span[2] = max(span[2], abs(float(cloud[:, 2].max())) + 10.0)
        ax3d.set_box_aspect(span)
        ax3d.set_zlim(0.0, max(float(cloud[:, 2].max()) * 1.15, 30.0))
        ax3d.view_init(elev=22, azim=-60)

    # Top view
    ax_top.set_aspect("equal")
    ax_top.set_xlabel("X fwd")
    ax_top.set_ylabel("Y left")
    ax_top.plot(mounts_xy[:, 0], mounts_xy[:, 1], "k.", ms=4)
    stance = frame.get("stance_ids") or []
    if len(stance) >= 3:
        pts = []
        for i in stance[:3]:
            fxy = frame["foot_xy"][i]
            pts.append(fxy[:2] if len(fxy) > 2 else fxy)
        xy = np.asarray(pts, dtype=float)
        poly = Polygon(xy, closed=True, fill=False, edgecolor="gray")
        ax_top.add_patch(poly)
    com = frame.get("com_xy", [0, 0])
    ax_top.plot(com[0], com[1], "r+", ms=10, label="COM")
    for lid_s, fxy in frame.get("foot_xy", {}).items():
        lid = int(lid_s)
        ax_top.plot(fxy[0], fxy[1], "o", color=COLORS[lid], ms=5)
    ax_top.set_title("Top: support / COM")

    # Hildebrand: footfall[leg, time] = 1 stance
    n_legs, n_t = footfall.shape
    ax_foot.set_xlim(0, 1)
    ax_foot.set_ylim(-0.5, n_legs - 0.5)
    ax_foot.set_yticks(range(n_legs))
    ax_foot.set_yticklabels(list(LEG_SHORT_NAMES))
    ax_foot.set_xlabel("cycle fraction")
    ax_foot.set_title("Hildebrand footfall (black=stance)")
    for i in range(n_legs):
        for j in range(n_t):
            if footfall[i, j] > 0.5:
                ax_foot.barh(i, 1.0 / n_t, left=j / n_t, height=0.6, color="black")
    ax_foot.axvline(t_norm, color="red", lw=1)


def build_footfall(anim_frames: list[dict[str, Any]], frames_per_cycle: int | None = None) -> np.ndarray:
    if not anim_frames:
        return np.zeros((6, 1))
    if frames_per_cycle is None:
        # detect cycle length from first cycle boundary if present
        frames_per_cycle = len(anim_frames)
    n = min(frames_per_cycle, len(anim_frames))
    mat = np.zeros((6, n))
    for j, fr in enumerate(anim_frames[:n]):
        roles = fr.get("roles") or {}
        for lid in range(6):
            role = roles.get(lid, roles.get(str(lid), "transfer"))
            mat[lid, j] = 1.0 if role == "support" else 0.0
    return mat


def _default_fps(anim_frames: list[dict[str, Any]]) -> float:
    """
    Comfortable playback FPS from frame timestamps.

    Pulse traj has few steps (150 ms each); realtime ≈ 6–7 fps feels rushed,
    so we play at half speed (≈ 3 fps) by default.
    """
    times = [float(fr.get("t_ms", 0.0)) for fr in anim_frames]
    dts = [times[i + 1] - times[i] for i in range(len(times) - 1) if times[i + 1] > times[i]]
    if dts:
        median_dt_ms = float(np.median(dts))
        realtime = 1000.0 / median_dt_ms
        return max(1.0, min(realtime * 0.5, 8.0))
    return 3.0


def animate_motion(
    anim_frames: list[dict[str, Any]],
    mounts_xy: np.ndarray,
    *,
    out_gif: Path | None = None,
    show: bool = False,
    title: str = "Hexapod motion",
    fps: float | None = None,
) -> Path | None:
    if not anim_frames:
        raise ValueError("No animation frames")

    # normalize foot_xy keys to int in a working copy
    frames = []
    for fr in anim_frames:
        f = dict(fr)
        fxy = {}
        for k, v in (fr.get("foot_xy") or {}).items():
            fxy[int(k)] = v
        f["foot_xy"] = fxy
        roles = {}
        for k, v in (fr.get("roles") or {}).items():
            roles[int(k)] = v
        f["roles"] = roles
        chains = {}
        for k, v in (fr.get("chains") or {}).items():
            chains[int(k)] = v
        f["chains"] = chains
        frames.append(f)

    play_fps = float(fps) if fps is not None else _default_fps(anim_frames)
    interval_ms = int(round(1000.0 / play_fps))

    footfall = build_footfall(frames)
    fig = plt.figure(figsize=(12, 8))
    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    ax_top = fig.add_subplot(2, 2, 2)
    ax_foot = fig.add_subplot(2, 1, 2)

    def _update(i: int):
        fr = frames[i]
        t_norm = (i % footfall.shape[1]) / max(footfall.shape[1], 1)
        _draw_frame(ax3d, ax_top, ax_foot, fr, mounts_xy, footfall, t_norm, title)
        return []

    anim = FuncAnimation(
        fig, _update, frames=len(frames), interval=interval_ms, blit=False
    )

    out_path = None
    if out_gif is not None:
        out_gif = Path(out_gif)
        out_gif.parent.mkdir(parents=True, exist_ok=True)
        anim.save(str(out_gif), writer=PillowWriter(fps=play_fps))
        out_path = out_gif

    if show:
        plt.show()
    else:
        plt.close(fig)

    return out_path
