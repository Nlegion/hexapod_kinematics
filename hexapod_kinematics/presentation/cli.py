"""CLI: extract kinematics from KOMPAS CAD; simulate gait offline; visualize."""

from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path

import numpy as np
import yaml

from hexapod_kinematics.core.config_loader import ConfigError, load_config
from hexapod_kinematics.core.logging import setup_logging

logger = logging.getLogger(__name__)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="hexapod_kinematics",
        description="KOMPAS kinematics extract + hexapod motion calculator",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    extract = sub.add_parser("extract", help="Scan CAD and write export artifacts")
    extract.add_argument(
        "--config",
        type=Path,
        default=Path("extractor_config.yml"),
        help="Path to extractor_config.yml",
    )
    extract.add_argument(
        "--export-dir",
        type=Path,
        default=Path("export"),
        help="Directory for JSON/header output",
    )
    extract.add_argument("--json", action="store_true", help="Write kinematics_996.json")
    extract.add_argument("--header", action="store_true", help="Write generated header")
    extract.add_argument("--gabarit", action="store_true", help="Collect gabarits")
    extract.add_argument("--mount-map", type=Path, default=None)
    extract.add_argument("-v", "--verbose", action="store_true")

    sim = sub.add_parser("simulate", help="Offline pulse/IK gait simulation + logs")
    sim.add_argument(
        "--json",
        type=Path,
        default=Path("export/kinematics_996.json"),
        help="Kinematics JSON",
    )
    sim.add_argument(
        "--gait",
        type=Path,
        default=Path("config/hexapod_gait.yml"),
        help="Gait YAML (firmware traj + IK params)",
    )
    sim.add_argument(
        "--masses",
        type=Path,
        default=Path("config/masses_996.yml"),
        help="Mass model YAML",
    )
    sim.add_argument(
        "--mode",
        choices=("pulse", "pulse_raw", "pulse_aligned", "ik", "compare"),
        default="pulse",
        help="pulse aliases to pulse_aligned; compare = aligned vs ik",
    )
    sim.add_argument(
        "--direction",
        choices=("forward", "backward", "turn_left", "turn_right"),
        default="forward",
    )
    sim.add_argument("--cycles", type=int, default=2)
    sim.add_argument("--log-dir", type=Path, default=Path("export/logs"))
    sim.add_argument(
        "--body-height",
        type=float,
        default=None,
        help="Override body height (mm) for mounts/COM/support/viz",
    )
    sim.add_argument(
        "--scale-pulse-to-stride",
        type=float,
        default=None,
        help="Scale coxa pulse offsets to target stride (mm)",
    )
    sim.add_argument("--animate", action="store_true")
    sim.add_argument("--out-gif", type=Path, default=None)
    sim.add_argument(
        "--fps",
        type=float,
        default=None,
        help="GIF/playback FPS (default: half realtime from frame dt)",
    )
    sim.add_argument(
        "--interactive",
        action="store_true",
        help="Matplotlib slider over cached frames (no re-sim)",
    )
    sim.add_argument("--show", action="store_true")
    sim.add_argument("--torques", action="store_true", help="Write Lagrange torque CSV")
    sim.add_argument("--no-report", action="store_true")
    sim.add_argument("-v", "--verbose", action="store_true")

    viz = sub.add_parser("visualize", help="Static kinematics stick figure from JSON")
    viz.add_argument(
        "--json",
        type=Path,
        default=Path("export/kinematics_996.json"),
    )
    viz.add_argument("--out", type=Path, default=None)
    viz.add_argument("--show", action="store_true")
    viz.add_argument(
        "--body-height",
        type=float,
        default=None,
        help="Place hips at this Z (mm) for display",
    )
    viz.add_argument("-v", "--verbose", action="store_true")

    sync = sub.add_parser(
        "sync-hexapod",
        help="Patch Arduino Config.h with CAD lengths + scan dependents",
    )
    sync.add_argument(
        "--json",
        type=Path,
        default=Path("export/kinematics_996.json"),
    )
    sync.add_argument(
        "--config-h",
        type=Path,
        default=Path(r"P:\Arduino\hexapod\src\core\Config.h"),
    )
    sync.add_argument("-v", "--verbose", action="store_true")

    return parser


def _cmd_extract(args: argparse.Namespace) -> int:
    from hexapod_kinematics.application.pipeline import run_extract
    from hexapod_kinematics.infrastructure.kompas.session import KompasError

    write_json = bool(args.json)
    write_header = bool(args.header)
    if not write_json and not write_header:
        write_json = True
        write_header = True

    config_path = args.config if args.config.is_file() else None
    if args.config and not args.config.is_file():
        logger.warning("config_missing path=%s; using built-in defaults", args.config)

    try:
        config = load_config(config_path)
        outputs = run_extract(
            config=config,
            config_path=config_path,
            export_dir=args.export_dir,
            write_json=write_json,
            write_header=write_header,
            collect_gabarit=bool(args.gabarit),
            mount_map_path=args.mount_map,
        )
    except (ConfigError, KompasError) as exc:
        logger.error("%s", exc)
        return 1
    except Exception:  # noqa: BLE001
        logger.exception("extract_failed")
        return 2

    sys.stdout.write(" ".join(str(p) for p in outputs.values()) + "\n")
    return 0


def _anim_key(mode: str, result: dict) -> str:
    if mode in ("pulse", "pulse_aligned"):
        return "pulse_aligned"
    if mode == "pulse_raw":
        return "pulse_raw"
    if mode == "ik":
        return "ik"
    return "pulse_aligned"


def _cmd_simulate(args: argparse.Namespace) -> int:
    from hexapod_kinematics.application.simulate_motion import run_simulation
    from hexapod_kinematics.domain.body_frame_kin import lengths_from_model
    from hexapod_kinematics.domain.dynamics_leg import compute_torques_for_log, write_torques_csv
    from hexapod_kinematics.domain.frames_world import (
        mounts_from_model,
        place_hips_above_ground,
        resolve_body_height_mm,
    )
    from hexapod_kinematics.domain.masses import load_masses_config
    from hexapod_kinematics.presentation.animate_motion import animate_motion
    from hexapod_kinematics.presentation.interactive_motion import interactive_motion

    try:
        result = run_simulation(
            json_path=args.json,
            gait_path=args.gait,
            mode=args.mode,
            cycles=args.cycles,
            direction=args.direction,
            log_dir=args.log_dir,
            scale_pulse_to_stride=args.scale_pulse_to_stride,
            body_height_mm=args.body_height,
            masses_path=args.masses,
            write_report_file=not bool(args.no_report),
        )
    except Exception:  # noqa: BLE001
        logger.exception("simulate_failed")
        return 2

    # Always print report to stdout
    sys.stdout.write(result.get("report_md", "") + "\n")
    paths = result.get("log_paths") or {}
    if paths:
        sys.stdout.write("logs: " + " ".join(str(p) for p in paths.values()) + "\n")

    if args.torques:
        branch = _anim_key(args.mode, result)
        if args.mode == "compare":
            branch = "pulse_aligned"
        if branch not in result and "ik" in result:
            branch = "ik"
        with args.json.open(encoding="utf-8") as fh:
            model = json.load(fh)
        masses = load_masses_config(args.masses)
        lengths = lengths_from_model(model)
        rows = compute_torques_for_log(
            result[branch]["legs"], lengths=lengths, masses_cfg=masses
        )
        tpath = Path(args.log_dir) / f"motion_{branch}_torques.csv"
        write_torques_csv(tpath, rows)
        sys.stdout.write(f"torques: {tpath}\n")

    need_viz = args.animate or args.show or args.out_gif or args.interactive
    if need_viz:
        with args.json.open(encoding="utf-8") as fh:
            model = json.load(fh)
        with args.gait.open(encoding="utf-8") as fh:
            gait = yaml.safe_load(fh)
        body_h = float(
            result.get("body_height_mm")
            or args.body_height
            or resolve_body_height_mm(gait, model)
        )
        mounts = place_hips_above_ground(mounts_from_model(model), body_h)
        mounts_xy = np.array([m.origin[:2] for m in mounts], dtype=float)

        if args.mode == "compare":
            for key in ("pulse_aligned", "ik"):
                anim = result[key]["anim"]
                if args.interactive and key == "ik":
                    interactive_motion(anim, mounts_xy, title=f"Hexapod {key}")
                gif = None
                if args.out_gif:
                    gif = args.out_gif.with_name(
                        args.out_gif.stem + f"_{key}" + args.out_gif.suffix
                    )
                if args.animate or args.out_gif or (args.show and key == "ik"):
                    animate_motion(
                        anim,
                        mounts_xy,
                        out_gif=gif,
                        show=bool(args.show) and key == "ik",
                        title=f"Hexapod {key}",
                        fps=args.fps,
                    )
        else:
            key = _anim_key(args.mode, result)
            anim = result[key]["anim"]
            if args.interactive:
                interactive_motion(anim, mounts_xy, title=f"Hexapod {key}")
            if args.animate or args.out_gif or args.show:
                animate_motion(
                    anim,
                    mounts_xy,
                    out_gif=args.out_gif,
                    show=bool(args.show),
                    title=f"Hexapod {key}",
                    fps=args.fps,
                )

    return 0


def _cmd_visualize(args: argparse.Namespace) -> int:
    from hexapod_kinematics.presentation.visualize_kinematics import render

    try:
        with args.json.open(encoding="utf-8") as fh:
            model = json.load(fh)
        render(
            model,
            out_path=args.out,
            show=bool(args.show) or args.out is None,
            body_height_mm=args.body_height,
        )
    except Exception:  # noqa: BLE001
        logger.exception("visualize_failed")
        return 2
    return 0


def _cmd_sync_hexapod(args: argparse.Namespace) -> int:
    from hexapod_kinematics.application.sync_hexapod import patch_config_h, report_config_diff
    from hexapod_kinematics.domain.body_frame_kin import lengths_from_model

    try:
        with args.json.open(encoding="utf-8") as fh:
            model = json.load(fh)
        lengths = lengths_from_model(model)
        L = model.get("lengths_mm") or {}
        tibia_eff = float(L.get("tibia_effective", lengths.tibia))
        pad = float((model.get("foot_pad") or {}).get("protrusion_mm", 2.0))
        diff = report_config_diff(lengths, model)
        sys.stdout.write(json.dumps(diff, indent=2) + "\n")
        warns = patch_config_h(
            args.config_h,
            coxa=lengths.coxa,
            femur=lengths.femur,
            tibia=float(L.get("tibia", lengths.tibia)),
            tibia_effective=tibia_eff,
            foot_pad_protrusion=pad,
        )
        for w in warns:
            sys.stdout.write(f"WARNING: {w}\n")
    except Exception:  # noqa: BLE001
        logger.exception("sync_hexapod_failed")
        return 2
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    verbose = bool(getattr(args, "verbose", False))
    setup_logging(level=logging.DEBUG if verbose else logging.INFO)

    if args.command == "extract":
        return _cmd_extract(args)
    if args.command == "simulate":
        return _cmd_simulate(args)
    if args.command == "visualize":
        return _cmd_visualize(args)
    if args.command == "sync-hexapod":
        return _cmd_sync_hexapod(args)
    parser.error(f"unknown command {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
