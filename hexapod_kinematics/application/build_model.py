"""Build kinematics export model from link + body extractions."""

from __future__ import annotations

from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from hexapod_kinematics import __version__
from hexapod_kinematics.application.extract_body import BodyExtraction
from hexapod_kinematics.application.extract_links import LinkExtraction
from hexapod_kinematics.core.constants import body_frame as body_frame_const
from hexapod_kinematics.core.constants import foot_pad as foot_pad_const
from hexapod_kinematics.core.constants import runtime, servo
from hexapod_kinematics.core.constants import world_frame as world_frame_const


def _avg(values: list[float]) -> float | None:
    if not values:
        return None
    return sum(values) / len(values)


def build_kinematics_model(
    *,
    config: dict[str, Any],
    config_path: Path | None,
    links: list[LinkExtraction],
    body: BodyExtraction,
    gabarits: dict[str, Any] | None = None,
) -> dict[str, Any]:
    cad_root = Path(config["roots"]["cad"])
    generated_at = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    yaw_unit = str(config.get("body_frame", {}).get("yaw_unit", "deg"))
    servo_cfg = config.get("servo", {})

    sources: list[dict[str, str]] = []
    link_entries: list[dict[str, Any]] = []
    by_role_mirror: dict[str, dict[str, float]] = defaultdict(dict)
    by_role_synth: dict[str, bool] = defaultdict(bool)
    global_warnings: list[str] = []

    for item in links:
        entry: dict[str, Any] = {
            "folder": item.folder,
            "role": item.role,
            "mirror": item.mirror,
            "source": str(item.source_path) if item.source_path else None,
            "sha256": item.sha256,
            "missing": item.missing,
            "warnings": item.warnings,
        }
        if item.source_path and item.sha256:
            sources.append(
                {
                    "path": str(item.source_path),
                    "role": f"link:{item.role}:{item.mirror or ''}",
                    "sha256": item.sha256,
                }
            )
        if item.result is not None:
            entry.update(
                {
                    "length_mm": item.result.length_mm,
                    "axis_alignment_ok": item.result.axis_alignment_ok,
                    "angle_deg": item.result.angle_deg,
                    "lateral_mm": item.result.lateral_mm,
                    "synthesized_out": item.result.synthesized_out,
                    "ambiguous": item.result.ambiguous,
                    "lcs_in": (
                        item.result.frame_in.to_dict()
                        if item.result.frame_in
                        else None
                    ),
                    "lcs_out": (
                        item.result.frame_out.to_dict()
                        if item.result.frame_out
                        else None
                    ),
                }
            )
            if item.mirror and not item.missing:
                by_role_mirror[item.role][item.mirror] = item.result.length_mm
                if item.result.synthesized_out:
                    by_role_synth[item.role] = True
        else:
            entry["length_mm"] = None
        link_entries.append(entry)
        global_warnings.extend(item.warnings)

    lengths: dict[str, Any] = {}
    for role in ("coxa", "femur", "tibia"):
        mirrors = by_role_mirror.get(role, {})
        for mirror, value in mirrors.items():
            lengths[f"{role}_{mirror}"] = value
        avg = _avg(list(mirrors.values()))
        lengths[role] = avg

    femur_source = "measured"
    femur_synthesized_approximate = bool(by_role_synth.get("femur"))
    override = servo_cfg.get("femur_override_length_mm")
    if override is not None:
        override_f = float(override)
        lengths["femur"] = override_f
        lengths["femur_A"] = override_f
        lengths["femur_B"] = override_f
        femur_source = "override"
        femur_synthesized_approximate = False
        global_warnings.append(
            f"femur length overridden to {override_f} mm via config"
        )
    elif femur_synthesized_approximate:
        femur_source = "datasheet_height"
        axis_len = float(servo_cfg.get("mg996r_axis_length_mm", 42.9))
        global_warnings.append(
            f"femur_synthesized_approximate from MG996R height {axis_len} mm "
            f"({servo_cfg.get('datasheet_ref', servo.DATASHEET_REF)})"
        )

    leg_names = body_frame_const.LEG_NAMES
    body_mounts = []
    for m in sorted(body.mounts, key=lambda x: x.leg_id):
        body_mounts.append(
            {
                "leg_id": m.leg_id,
                "leg_name": leg_names[m.leg_id] if 0 <= m.leg_id < 6 else "UNKNOWN",
                "lcs_name": m.lcs_name,
                "component": m.component,
                "component_index": m.component_index,
                "key": m.key,
                "assignment": m.assignment,
                "yaw": m.yaw_value,
                "yaw_unit": yaw_unit,
                "coxa_axis_level_ok": m.coxa_axis_level_ok,
                "origin_mm": m.frame.origin.as_list(),
                "axis_x": m.frame.axis_x.as_list(),
                "axis_y": m.frame.axis_y.as_list(),
                "axis_z": m.frame.axis_z.as_list(),
                "ambiguous": m.ambiguous,
            }
        )
    sources.extend(body.sources)
    global_warnings.extend(body.warnings)

    femur = lengths.get("femur")
    tibia = lengths.get("tibia")
    foot_cfg = config.get("foot_pad", {})
    protrusion = float(
        foot_cfg.get("protrusion_mm", foot_pad_const.FOOT_PAD_PROTRUSION_MM)
    )
    tibia_effective = None
    if tibia is not None:
        tibia_effective = float(tibia) + protrusion
        lengths["tibia_effective"] = tibia_effective

    max_reach = None
    if femur is not None and tibia_effective is not None:
        max_reach = float(femur) + float(tibia_effective)
    elif femur is not None and tibia is not None:
        max_reach = float(femur) + float(tibia)

    model: dict[str, Any] = {
        "meta": {
            "tool_name": runtime.TOOL_NAME,
            "tool_version": __version__,
            "generated_at": generated_at,
            "cad_root": str(cad_root),
            "config_path": str(config_path.resolve()) if config_path else None,
            "yaw_unit": yaw_unit,
            "body_frame": {
                "forward_axis": config.get("body_frame", {}).get("forward_axis"),
                "up_axis": config.get("body_frame", {}).get("up_axis"),
                "cad_to_world_rotation": config.get("body_frame", {}).get(
                    "cad_to_world_rotation",
                    world_frame_const.DEFAULT_CAD_TO_WORLD_ROTATION,
                ),
                "body_forward_axis": config.get("body_frame", {}).get(
                    "body_forward_axis_world",
                    world_frame_const.DEFAULT_BODY_FORWARD_AXIS_WORLD,
                ),
                "body_up_axis": config.get("body_frame", {}).get(
                    "body_up_axis_world",
                    world_frame_const.DEFAULT_BODY_UP_AXIS_WORLD,
                ),
            },
        },
        "sources": sources,
        "lengths_mm": lengths,
        "femur_source": femur_source,
        "femur_synthesized_approximate": femur_synthesized_approximate,
        "servo_datasheet": {
            "body_length_mm": float(servo_cfg.get("body_length_mm", 40.7)),
            "body_width_mm": float(servo_cfg.get("body_width_mm", 19.7)),
            "body_height_mm": float(servo_cfg.get("body_height_mm", 42.9)),
            "axis_length_mm": float(servo_cfg.get("mg996r_axis_length_mm", 42.9)),
            "ref": servo_cfg.get("datasheet_ref", servo.DATASHEET_REF),
        },
        "foot_pad": {
            "part": foot_cfg.get("part", foot_pad_const.FOOT_PAD_PART),
            "diameter_mm": float(
                foot_cfg.get("diameter_mm", foot_pad_const.FOOT_PAD_DIAMETER_MM)
            ),
            "height_mm": float(
                foot_cfg.get("height_mm", foot_pad_const.FOOT_PAD_HEIGHT_MM)
            ),
            "recess_mm": float(
                foot_cfg.get("recess_mm", foot_pad_const.FOOT_PAD_RECESS_MM)
            ),
            "protrusion_mm": protrusion,
            "note": (
                "Silicone tip pad recessed into tibia; "
                "protrusion raises stance / effective tip length by 2 mm"
            ),
        },
        "reach": {
            "max_reach_mm": max_reach,
            "min_useful_mm": body_frame_const.MIN_USEFUL_REACH_MM,
            "includes_foot_pad_protrusion": True,
        },
        "links": link_entries,
        "body_mounts": body_mounts,
        "unmapped_lcs": body.unmapped_lcs,
        "rejected_mount_candidates": body.rejected_candidates,
        "warnings": global_warnings,
    }
    if gabarits:
        model["gabarits"] = gabarits
        # Prefer assembly gabarit as meta.body_size_mm for resolve_body_height_mm
        for key, box in gabarits.items():
            if not isinstance(box, dict):
                continue
            if "body" not in str(key).lower() and "assembly" not in str(key).lower():
                continue
            size = box.get("size_mm") or box.get("dimensions_mm")
            if size and len(size) >= 3:
                model.setdefault("meta", {})
                model["meta"]["body_size_mm"] = [float(x) for x in size[:3]]
                break
    return model
