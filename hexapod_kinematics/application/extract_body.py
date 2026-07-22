"""Extract body leg mounts from assembly .a3d into assembly frame."""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from hexapod_kinematics.application.body_extract_io import (
    candidate_key,
    leg_id_for_lcs,
    payload_from_frame,
    pick_assembly,
    resolve_component_path,
)
from hexapod_kinematics.application.extract_links import file_sha256
from hexapod_kinematics.core.config_loader import load_mount_map_file
from hexapod_kinematics.domain.body_mounts import (
    MountCandidate,
    auto_assign_mounts,
    build_body_basis,
    convert_yaw,
    coxa_axis_vertical_ok,
    horizontal_extent_mm,
    mean_origin,
    yaw_angle_rad,
)
from hexapod_kinematics.domain.frames import Frame3D, Vec3
from hexapod_kinematics.infrastructure.kompas.documents import close_document, open_document
from hexapod_kinematics.infrastructure.kompas.lcs_reader import read_local_coordinate_systems
from hexapod_kinematics.infrastructure.kompas.matrices import iter_assembly_components
from hexapod_kinematics.infrastructure.kompas.session import KompasSession

logger = logging.getLogger(__name__)


@dataclass
class BodyMount:
    leg_id: int
    lcs_name: str
    component: str
    component_index: int
    frame: Frame3D
    assignment: str
    yaw_value: float
    coxa_axis_level_ok: bool
    ambiguous: bool = False
    key: str = ""


@dataclass
class BodyExtraction:
    assembly_path: Path | None
    sha256: str | None
    mounts: list[BodyMount] = field(default_factory=list)
    unmapped_lcs: list[dict[str, Any]] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    sources: list[dict[str, str]] = field(default_factory=list)
    rejected_candidates: list[dict[str, Any]] = field(default_factory=list)


def extract_body_mounts(
    session: KompasSession,
    cad_root: Path,
    config: dict[str, Any],
    *,
    mount_map_override: dict[str, int] | None = None,
) -> BodyExtraction:
    body_meta = None
    body_folder_name = None
    for folder_name, meta in config["folder_roles"].items():
        if meta.get("role") == "body":
            body_meta = meta
            body_folder_name = folder_name
            break

    result = BodyExtraction(assembly_path=None, sha256=None)
    if body_meta is None or body_folder_name is None:
        result.warnings.append("no body folder_roles entry")
        return result

    body_folder = cad_root / body_folder_name
    if not body_folder.is_dir():
        result.warnings.append(f"body folder missing: {body_folder}")
        return result

    assembly = pick_assembly(
        body_folder,
        list(body_meta.get("assembly_candidates", [])),
    )
    if assembly is None:
        result.warnings.append(f"no .a3d assembly in {body_folder}")
        return result

    result.assembly_path = assembly
    result.sha256 = file_sha256(assembly)
    result.sources.append(
        {"path": str(assembly), "role": "body_assembly", "sha256": result.sha256}
    )

    kompas_cfg = config["kompas"]
    doc = open_document(
        session,
        assembly,
        open_retries=int(kompas_cfg["open_retries"]),
        retry_delay_sec=float(kompas_cfg["retry_delay_sec"]),
    )
    try:
        components = iter_assembly_components(doc)
    finally:
        close_document(doc)

    role_map = config["lcs_role_map"]
    leg_mount_map = {str(k): int(v) for k, v in config["leg_mount_map"].items()}
    external_map: dict[str, int] = {}
    map_path = config.get("mount_map_path")
    if mount_map_override is not None:
        external_map = mount_map_override
    elif map_path:
        external_map = load_mount_map_file(Path(str(map_path)))

    body_cfg = config.get("body_frame", {})
    basis = build_body_basis(
        Vec3.from_list(body_cfg["forward_axis"]),
        Vec3.from_list(body_cfg["up_axis"]),
    )
    yaw_unit = str(body_cfg.get("yaw_unit", "deg"))
    coxa_tol = float(body_cfg.get("coxa_up_dot_tol", 0.15))
    auto_cfg = config.get("body_mount_auto", {})
    candidate_names = set(auto_cfg.get("candidate_lcs_names", ["LCS_in"]))

    # Collect all world frames first
    collected: list[tuple[int, str, Path, Frame3D]] = []
    for component_index, component in enumerate(components):
        part_path = resolve_component_path(
            cad_root,
            body_folder,
            component.file_path,
            component.name,
        )
        if part_path is None:
            msg = (
                f"component file not resolved for {component.name!r} "
                f"(path={component.file_path})"
            )
            logger.warning(msg)
            result.warnings.append(msg)
            continue

        sha = file_sha256(part_path)
        result.sources.append(
            {
                "path": str(part_path),
                "role": f"body_component:{component.name}",
                "sha256": sha,
            }
        )

        part_doc = open_document(
            session,
            part_path,
            open_retries=int(kompas_cfg["open_retries"]),
            retry_delay_sec=float(kompas_cfg["retry_delay_sec"]),
        )
        try:
            local_frames = read_local_coordinate_systems(part_doc)
        finally:
            close_document(part_doc)

        seen_in_component: dict[str, int] = {}
        for frame in local_frames:
            count = seen_in_component.get(frame.name, 0) + 1
            seen_in_component[frame.name] = count
            ambiguous = count > 1
            if ambiguous:
                msg = (
                    f"duplicate LCS name {frame.name!r} in component "
                    f"{component.name!r}; using first occurrence policy"
                )
                logger.warning(msg)
                result.warnings.append(msg)

            world = component.placement.transform_frame(frame)
            if ambiguous:
                world = Frame3D(
                    name=world.name,
                    origin=world.origin,
                    axis_x=world.axis_x,
                    axis_y=world.axis_y,
                    axis_z=world.axis_z,
                    ambiguous=True,
                    synthesized=world.synthesized,
                )
            collected.append((component_index, component.name, part_path, world))

    assigned_legs: set[int] = set()
    used_keys: set[str] = set()

    def _add_mount(
        *,
        leg_id: int,
        frame: Frame3D,
        component: str,
        component_index: int,
        part_path: Path,
        assignment: str,
    ) -> None:
        key = candidate_key(component_index, frame.name)
        yaw_rad = yaw_angle_rad(frame.origin, Vec3(0, 0, 0), basis)
        # yaw relative to body origin (0,0,0) is fine for explicit; recompute vs mean later for auto
        level_ok = coxa_axis_vertical_ok(frame.axis_z, basis.up, coxa_tol)
        if not level_ok:
            msg = (
                f"mount {key} leg_id={leg_id}: coxa Z not aligned with body up"
            )
            logger.warning(msg)
            result.warnings.append(msg)
        result.mounts.append(
            BodyMount(
                leg_id=leg_id,
                lcs_name=frame.name,
                component=component,
                component_index=component_index,
                frame=frame,
                assignment=assignment,
                yaw_value=convert_yaw(yaw_rad, yaw_unit),
                coxa_axis_level_ok=level_ok,
                ambiguous=frame.ambiguous,
                key=key,
            )
        )
        assigned_legs.add(leg_id)
        used_keys.add(key)

    # Pass 1: explicit mount_leg_* / leg_mount_map
    for component_index, component_name, part_path, world in collected:
        leg_id = leg_id_for_lcs(world.name, role_map, leg_mount_map)
        if leg_id is None:
            continue
        if leg_id in assigned_legs:
            result.warnings.append(
                f"leg_id {leg_id} already mapped; ignoring explicit {world.name!r}"
            )
            continue
        _add_mount(
            leg_id=leg_id,
            frame=world,
            component=component_name,
            component_index=component_index,
            part_path=part_path,
            assignment="explicit",
        )

    # Pass 2: external mount map (keys: "index:name" or name)
    for component_index, component_name, part_path, world in collected:
        key = candidate_key(component_index, world.name)
        if key in used_keys:
            continue
        leg_id = None
        if key in external_map:
            leg_id = external_map[key]
        elif world.name in external_map:
            leg_id = external_map[world.name]
        if leg_id is None:
            continue
        if leg_id in assigned_legs:
            result.warnings.append(
                f"leg_id {leg_id} already mapped; ignoring map key {key!r}"
            )
            continue
        _add_mount(
            leg_id=leg_id,
            frame=world,
            component=component_name,
            component_index=component_index,
            part_path=part_path,
            assignment="mount_map_file",
        )

    # Pass 3: auto for remaining LCS_in-like candidates
    if (
        auto_cfg.get("enabled", True)
        and len(assigned_legs) < 6
    ):
        auto_pool: list[MountCandidate] = []
        for component_index, component_name, part_path, world in collected:
            key = candidate_key(component_index, world.name)
            if key in used_keys:
                continue
            if world.name not in candidate_names:
                result.unmapped_lcs.append(
                    payload_from_frame(
                        world,
                        component=component_name,
                        component_index=component_index,
                        part_path=part_path,
                    )
                )
                continue
            auto_pool.append(
                MountCandidate(
                    key=key,
                    frame=world,
                    component=component_name,
                    component_index=component_index,
                    file_path=str(part_path),
                )
            )

        if auto_pool:
            frames = [c.frame for c in auto_pool]
            center = mean_origin(frames)
            extent = horizontal_extent_mm(
                [f.origin for f in frames],
                center,
                basis,
            )
            mode = auto_cfg.get("radius_mode", "fraction_of_extent")
            if mode == "absolute_mm" or extent <= 1e-6:
                threshold = float(auto_cfg.get("radius_absolute_mm", 50.0))
            else:
                fraction = float(auto_cfg.get("radius_fraction", 0.25))
                threshold = extent * fraction

            logger.info(
                "mount_auto_filter center=%s extent=%.2f threshold=%.2f pool=%s",
                center.as_list(),
                extent,
                threshold,
                len(auto_pool),
            )

            # Only assign free leg slots: filter leg_order to free ids
            full_order = [int(x) for x in auto_cfg.get("leg_order_from_anchor", list(range(6)))]
            free_order = [leg_id for leg_id in full_order if leg_id not in assigned_legs]

            assignments, rejected, auto_warnings = auto_assign_mounts(
                auto_pool,
                basis=basis,
                center=center,
                radius_threshold_mm=threshold,
                sweep=str(auto_cfg.get("sweep", "cw")),
                prefer_side=str(
                    auto_cfg.get("start_anchor", {}).get(
                        "prefer_side",
                        "negative_right",
                    )
                ),
                leg_order_from_anchor=free_order,
                yaw_unit=yaw_unit,
                coxa_up_dot_tol=coxa_tol,
                expect_vertical=True,
            )
            result.warnings.extend(auto_warnings)

            # Recompute yaw relative to center for assigned mounts
            for item in assignments:
                if item.leg_id in assigned_legs:
                    continue
                yaw_rad = yaw_angle_rad(item.candidate.frame.origin, center, basis)
                result.mounts.append(
                    BodyMount(
                        leg_id=item.leg_id,
                        lcs_name=item.candidate.frame.name,
                        component=item.candidate.component,
                        component_index=item.candidate.component_index,
                        frame=item.candidate.frame,
                        assignment=item.assignment,
                        yaw_value=convert_yaw(yaw_rad, yaw_unit),
                        coxa_axis_level_ok=item.coxa_axis_level_ok,
                        ambiguous=item.candidate.frame.ambiguous,
                        key=item.candidate.key,
                    )
                )
                assigned_legs.add(item.leg_id)
                used_keys.add(item.candidate.key)

            for cand in rejected:
                result.rejected_candidates.append(
                    {
                        "key": cand.key,
                        "radius_mm": cand.radius_mm,
                        "yaw_rad": cand.yaw_rad,
                        "origin_mm": cand.frame.origin.as_list(),
                        "reason": "radius_or_extra",
                    }
                )
                result.unmapped_lcs.append(
                    payload_from_frame(
                        cand.frame,
                        component=cand.component,
                        component_index=cand.component_index,
                        part_path=Path(cand.file_path),
                    )
                )

            # Remaining unassigned from pool
            assigned_keys = {m.key for m in result.mounts}
            for cand in auto_pool:
                if cand.key not in assigned_keys and cand.key not in {
                    r["key"] for r in result.rejected_candidates
                }:
                    result.unmapped_lcs.append(
                        payload_from_frame(
                            cand.frame,
                            component=cand.component,
                            component_index=cand.component_index,
                            part_path=Path(cand.file_path),
                        )
                    )

    # Anything collected but never used
    for component_index, component_name, part_path, world in collected:
        key = candidate_key(component_index, world.name)
        if key in used_keys:
            continue
        already = any(
            u.get("key") == key for u in result.unmapped_lcs
        )
        if not already:
            result.unmapped_lcs.append(
                payload_from_frame(
                    world,
                    component=component_name,
                    component_index=component_index,
                    part_path=part_path,
                )
            )

    # Fix yaw for explicit mounts using body center of all mounts
    if result.mounts:
        center = mean_origin([m.frame for m in result.mounts])
        for mount in result.mounts:
            yaw_rad = yaw_angle_rad(mount.frame.origin, center, basis)
            mount.yaw_value = convert_yaw(yaw_rad, yaw_unit)

    if len(result.mounts) < 6:
        result.warnings.append(
            f"body mounts incomplete: {len(result.mounts)}/6; "
            "see unmapped_lcs / rejected_candidates"
        )
    return result
