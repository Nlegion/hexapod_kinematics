"""Extract link lengths from leaf .m3d files only."""

from __future__ import annotations

import hashlib
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

from hexapod_kinematics.domain.lcs_roles import index_frames_by_role
from hexapod_kinematics.domain.link_length import LinkLengthResult, compute_link_length
from hexapod_kinematics.infrastructure.kompas.documents import close_document, open_document
from hexapod_kinematics.infrastructure.kompas.lcs_reader import read_local_coordinate_systems
from hexapod_kinematics.infrastructure.kompas.session import KompasSession

logger = logging.getLogger(__name__)


@dataclass
class LinkExtraction:
    folder: str
    role: str
    mirror: str | None
    source_path: Path | None
    sha256: str | None
    result: LinkLengthResult | None
    warnings: list[str] = field(default_factory=list)
    missing: list[str] = field(default_factory=list)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _pick_leaf(folder: Path, candidates: list[str]) -> Path | None:
    for name in candidates:
        path = folder / name
        if path.is_file():
            return path
    # fallback: any .m3d preferring base.m3d
    m3d_files = sorted(folder.glob("*.m3d"))
    for path in m3d_files:
        if path.name.lower() == "base.m3d":
            return path
    return m3d_files[0] if m3d_files else None


def extract_link_from_part(
    session: KompasSession,
    part_path: Path,
    config: dict[str, Any],
) -> tuple[LinkLengthResult, list[str]]:
    kompas_cfg = config["kompas"]
    doc = open_document(
        session,
        part_path,
        open_retries=int(kompas_cfg["open_retries"]),
        retry_delay_sec=float(kompas_cfg["retry_delay_sec"]),
    )
    try:
        frames = read_local_coordinate_systems(doc)
        by_role, role_warnings = index_frames_by_role(
            frames,
            config["lcs_role_map"],
        )
        frame_in = by_role.get("in")
        frame_out = by_role.get("out")
        synthesize = None
        if frame_out is None and frame_in is not None:
            synthesize = float(config["servo"]["mg996r_axis_length_mm"])
        result = compute_link_length(
            frame_in,
            frame_out,
            angle_tol_deg=float(config["tolerances"]["angle_tol_deg"]),
            lateral_tol_mm=float(config["tolerances"]["lateral_tol_mm"]),
            synthesize_length_mm=synthesize,
        )
        return result, role_warnings
    finally:
        close_document(doc)


def extract_all_links(
    session: KompasSession,
    cad_root: Path,
    config: dict[str, Any],
) -> list[LinkExtraction]:
    extractions: list[LinkExtraction] = []
    folder_roles: dict[str, Any] = config["folder_roles"]

    for folder_name, meta in folder_roles.items():
        role = meta.get("role")
        if role not in ("coxa", "femur", "tibia"):
            continue
        folder = cad_root / folder_name
        warnings: list[str] = []
        if not folder.is_dir():
            msg = f"folder missing: {folder}"
            logger.warning(msg)
            extractions.append(
                LinkExtraction(
                    folder=folder_name,
                    role=role,
                    mirror=meta.get("mirror"),
                    source_path=None,
                    sha256=None,
                    result=None,
                    warnings=[msg],
                    missing=["in", "out"],
                )
            )
            continue

        leaf = _pick_leaf(folder, list(meta.get("leaf_candidates", [])))
        if leaf is None:
            msg = f"no .m3d leaf in {folder}"
            logger.warning(msg)
            extractions.append(
                LinkExtraction(
                    folder=folder_name,
                    role=role,
                    mirror=meta.get("mirror"),
                    source_path=None,
                    sha256=None,
                    result=None,
                    warnings=[msg],
                    missing=["in", "out"],
                )
            )
            continue

        logger.info("extract_link folder=%s leaf=%s", folder_name, leaf)
        result, role_warnings = extract_link_from_part(session, leaf, config)
        warnings.extend(role_warnings)
        warnings.extend(result.warnings)
        extractions.append(
            LinkExtraction(
                folder=folder_name,
                role=role,
                mirror=meta.get("mirror"),
                source_path=leaf,
                sha256=file_sha256(leaf),
                result=result,
                warnings=warnings,
                missing=list(result.missing),
            )
        )
    return extractions


# Typing helper for tests injecting a reader
LinkReader = Callable[[KompasSession, Path, dict[str, Any]], tuple[LinkLengthResult, list[str]]]
