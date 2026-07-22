"""Visualize hexapod kinematics from export/kinematics_996.json."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

LEG_SHORT = ("FR", "MR", "RR", "RL", "ML", "FL")
COLORS = ("#e74c3c", "#e67e22", "#f1c40f", "#2ecc71", "#3498db", "#9b59b6")


def _load(path: Path) -> dict:
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def _leg_points(
    origin: np.ndarray,
    center: np.ndarray,
    coxa: float,
    femur: float,
    tibia_print: float,
    pad: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Neutral standing chain.

    Returns (print_chain hip→coxa→knee→tip, pad_segment tip→contact, tibia_dir).
    """
    radial = origin - center
    radial[1] = 0.0
    norm = np.linalg.norm(radial)
    if norm < 1e-9:
        radial = np.array([1.0, 0.0, 0.0])
    else:
        radial = radial / norm
    up = np.array([0.0, 1.0, 0.0])
    down = -up

    hip = origin
    coxa_end = hip + radial * coxa
    femur_dir = (radial + down)
    femur_dir = femur_dir / np.linalg.norm(femur_dir)
    knee = coxa_end + femur_dir * femur
    tibia_dir = (0.35 * radial + down)
    tibia_dir = tibia_dir / np.linalg.norm(tibia_dir)
    tip = knee + tibia_dir * tibia_print
    contact = tip + tibia_dir * pad
    print_chain = np.vstack([hip, coxa_end, knee, tip])
    pad_seg = np.vstack([tip, contact])
    return print_chain, pad_seg, tibia_dir


def render(
    model: dict,
    out_path: Path | None = None,
    *,
    show: bool = False,
    body_height_mm: float | None = None,
) -> Path | None:
    mounts = sorted(model["body_mounts"], key=lambda m: m["leg_id"])
    lengths = model["lengths_mm"]
    coxa = float(lengths["coxa"])
    femur = float(lengths["femur"])
    tibia_print = float(lengths["tibia"])
    foot = model.get("foot_pad") or {}
    protrusion = float(foot.get("protrusion_mm", 0.0))
    recess = float(foot.get("recess_mm", 3.0))
    height = float(foot.get("height_mm", 5.0))
    part = str(foot.get("part", "RF-F12050"))
    tibia_eff = float(lengths.get("tibia_effective", tibia_print + protrusion))
    forward = np.array(model["meta"]["body_frame"]["forward_axis"], dtype=float)
    up = np.array(model["meta"]["body_frame"]["up_axis"], dtype=float)

    origins = np.array([m["origin_mm"] for m in mounts], dtype=float)
    if body_height_mm is not None:
        # CAD Y is up in assembly; display helper uses CAD axes — shift hip height
        origins = origins.copy()
        origins[:, 1] = float(body_height_mm)
    center = origins.mean(axis=0)

    fig = plt.figure(figsize=(14, 7.0), constrained_layout=True)
    ax_top = fig.add_subplot(1, 2, 1)
    ax_3d = fig.add_subplot(1, 2, 2, projection="3d")

    order = np.argsort([m["yaw"] for m in mounts])
    poly = origins[order][:, [0, 2]]
    poly = np.vstack([poly, poly[0]])
    ax_top.fill(poly[:, 0], poly[:, 1], color="#ecf0f1", alpha=0.9, zorder=0)
    ax_top.plot(poly[:, 0], poly[:, 1], color="#7f8c8d", lw=1.5, zorder=1)

    for mount, color in zip(mounts, COLORS, strict=True):
        o = np.array(mount["origin_mm"], dtype=float)
        leg_id = int(mount["leg_id"])
        ax_top.scatter(o[0], o[2], s=80, c=color, zorder=5, edgecolors="k", lw=0.5)
        ax_top.text(
            o[0] * 1.08,
            o[2] * 1.08,
            f"{LEG_SHORT[leg_id]} ({leg_id})\n{mount['yaw']:.0f}°",
            fontsize=8,
            ha="center",
            va="center",
            color=color,
            fontweight="bold",
        )
        radial = o - center
        radial[1] = 0
        radial = radial / (np.linalg.norm(radial) + 1e-12)
        tip = o + radial * coxa
        ax_top.plot([o[0], tip[0]], [o[2], tip[2]], color=color, lw=2, zorder=4)

    f_scale = 80.0
    ax_top.annotate(
        "",
        xy=(forward[0] * f_scale, forward[2] * f_scale),
        xytext=(0, 0),
        arrowprops={"arrowstyle": "->", "color": "#c0392b", "lw": 2},
    )
    ax_top.text(
        forward[0] * f_scale * 1.1,
        forward[2] * f_scale * 1.1,
        "FORWARD",
        color="#c0392b",
        fontsize=9,
        fontweight="bold",
    )
    ax_top.scatter([0], [0], c="k", s=30, zorder=6)
    ax_top.set_aspect("equal")
    ax_top.grid(True, alpha=0.3)
    ax_top.set_xlabel("CAD X (mm)")
    ax_top.set_ylabel("CAD Z = forward (mm)")
    ax_top.set_title("Top view — body mounts & coxa direction")
    ax_top.text(
        0.02,
        0.02,
        f"Tip pad {part}: Ø{foot.get('diameter_mm', 12):.0f}×{height:.0f} mm, "
        f"recess {recess:.0f} → +{protrusion:.0f} mm stance",
        transform=ax_top.transAxes,
        fontsize=8,
        color="#555555",
        va="bottom",
    )

    # --- 3D stick figure ---
    contact_ys: list[float] = []
    pad_plotted = False
    for mount, color in zip(mounts, COLORS, strict=True):
        o = np.array(mount["origin_mm"], dtype=float)
        print_chain, pad_seg, _ = _leg_points(
            o,
            center,
            coxa,
            femur,
            tibia_print,
            protrusion,
        )
        ax_3d.plot(
            print_chain[:, 0],
            print_chain[:, 2],
            print_chain[:, 1],
            "-o",
            color=color,
            lw=2.2,
            markersize=4,
            label=f"{LEG_SHORT[int(mount['leg_id'])]}",
        )
        # Foot pad protrusion (contact tip)
        ax_3d.plot(
            pad_seg[:, 0],
            pad_seg[:, 2],
            pad_seg[:, 1],
            "-",
            color="#1abc9c",
            lw=3.5,
            solid_capstyle="round",
            label="RF-F12050 (+2 mm)" if not pad_plotted else None,
        )
        pad_plotted = True
        ax_3d.scatter(
            [pad_seg[1, 0]],
            [pad_seg[1, 2]],
            [pad_seg[1, 1]],
            c="#16a085",
            s=28,
            zorder=6,
        )
        contact_ys.append(float(pad_seg[1, 1]))

        az = np.array(mount["axis_z"], dtype=float)
        tip = o + az * 25.0
        ax_3d.plot(
            [o[0], tip[0]],
            [o[2], tip[2]],
            [o[1], tip[1]],
            color="k",
            lw=1.0,
            alpha=0.7,
        )

    # Ground / contact plane at mean foot height
    if contact_ys:
        ground_y = float(np.mean(contact_ys))
        span = 160.0
        xx = np.array([-span, span, span, -span, -span])
        zz = np.array([-span, -span, span, span, -span])
        yy = np.full_like(xx, ground_y, dtype=float)
        ax_3d.plot(xx, zz, yy, color="#95a5a6", lw=1.0, alpha=0.8, linestyle="--")
        ax_3d.text(
            -span * 0.7,
            span * 0.55,
            ground_y - 8,
            f"contact plane (+{protrusion:.0f} mm pad)",
            color="#16a085",
            fontsize=8,
        )

    ax_3d.plot(
        list(origins[order, 0]) + [origins[order[0], 0]],
        list(origins[order, 2]) + [origins[order[0], 2]],
        list(origins[order, 1]) + [origins[order[0], 1]],
        color="#7f8c8d",
        lw=1.5,
    )
    ax_3d.quiver(
        0,
        0,
        0,
        forward[0] * 60,
        forward[2] * 60,
        forward[1] * 60,
        color="#c0392b",
        arrow_length_ratio=0.2,
        lw=2,
    )
    ax_3d.quiver(
        0,
        0,
        0,
        up[0] * 40,
        up[2] * 40,
        up[1] * 40,
        color="#27ae60",
        arrow_length_ratio=0.25,
        lw=2,
    )
    ax_3d.text(0, 70, 5, "F", color="#c0392b", fontsize=10)
    ax_3d.text(0, 5, 45, "UP", color="#27ae60", fontsize=10)

    ax_3d.set_xlabel("X")
    ax_3d.set_ylabel("Z (forward)")
    ax_3d.set_zlabel("Y (up)")
    ax_3d.set_title(
        "3D neutral stick figure\n"
        f"coxa={coxa:.1f}  femur={femur:.1f}  "
        f"tibia={tibia_print:.1f}+{protrusion:.0f}={tibia_eff:.1f} mm"
    )
    ax_3d.legend(loc="upper left", fontsize=8, ncol=2)

    all_pts = origins.copy()
    mins = all_pts.min(axis=0) - 80
    maxs = all_pts.max(axis=0) + 80
    ax_3d.set_xlim(mins[0], maxs[0])
    ax_3d.set_ylim(mins[2], maxs[2])
    zmin = mins[1] - 110
    if contact_ys:
        zmin = min(zmin, float(np.min(contact_ys)) - 20)
    ax_3d.set_zlim(zmin, maxs[1] + 40)

    femur_note = ""
    if model.get("femur_synthesized_approximate"):
        femur_note = " | femur ≈ datasheet 42.9"
    fig.suptitle(
        f"Hexapod 996 kinematics preview | {part} tip +{protrusion:.0f} mm"
        + femur_note,
        fontsize=12,
        fontweight="bold",
    )

    if out_path is not None:
        out_path = Path(out_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out_path, dpi=140)
    if show or out_path is None:
        plt.show()
    plt.close(fig)
    return out_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--json",
        type=Path,
        default=Path("export/kinematics_996.json"),
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("export/robot_kinematics_preview.png"),
    )
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args(argv)
    if not args.json.is_file():
        raise SystemExit(f"JSON not found: {args.json}")
    model = _load(args.json)
    path = render(model, args.out, show=args.show)
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
