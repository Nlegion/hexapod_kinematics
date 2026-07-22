"""Orchestrate full extract → export pipeline."""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

from hexapod_kinematics.application.build_model import build_kinematics_model
from hexapod_kinematics.application.export_header import write_kinematics_header
from hexapod_kinematics.application.export_json import write_kinematics_json
from hexapod_kinematics.application.extract_body import extract_body_mounts
from hexapod_kinematics.application.extract_links import extract_all_links, file_sha256
from hexapod_kinematics.core.config_loader import load_mount_map_file
from hexapod_kinematics.infrastructure.kompas.documents import close_document, open_document
from hexapod_kinematics.infrastructure.kompas.gabarit import read_gabarit
from hexapod_kinematics.infrastructure.kompas.lcs_reader import configure_tlb
from hexapod_kinematics.infrastructure.kompas.session import KompasSession

logger = logging.getLogger(__name__)


def run_extract(
    *,
    config: dict[str, Any],
    config_path: Path | None,
    export_dir: Path,
    write_json: bool,
    write_header: bool,
    collect_gabarit: bool = False,
    mount_map_path: Path | None = None,
) -> dict[str, Path]:
    cad_root = Path(config["roots"]["cad"])
    kompas_cfg = config["kompas"]
    outputs: dict[str, Path] = {}
    configure_tlb(str(kompas_cfg.get("bin_path") or ""))

    mount_override = None
    if mount_map_path is not None:
        mount_override = load_mount_map_file(mount_map_path)
    elif config.get("mount_map_path"):
        mount_override = load_mount_map_file(Path(str(config["mount_map_path"])))

    gabarits: dict[str, Any] = {}

    with KompasSession(
        progid=str(kompas_cfg["progid"]),
        connect_retries=int(kompas_cfg["connect_retries"]),
        retry_delay_sec=float(kompas_cfg["retry_delay_sec"]),
    ) as session:
        links = extract_all_links(session, cad_root, config)
        body = extract_body_mounts(
            session,
            cad_root,
            config,
            mount_map_override=mount_override,
        )

        if collect_gabarit:
            for item in links:
                if item.source_path is None:
                    continue
                doc = open_document(
                    session,
                    item.source_path,
                    open_retries=int(kompas_cfg["open_retries"]),
                    retry_delay_sec=float(kompas_cfg["retry_delay_sec"]),
                )
                try:
                    box = read_gabarit(doc)
                finally:
                    close_document(doc)
                if box:
                    gabarits[f"link:{item.folder}"] = {
                        **box,
                        "sha256": item.sha256 or file_sha256(item.source_path),
                    }
            if body.assembly_path is not None:
                doc = open_document(
                    session,
                    body.assembly_path,
                    open_retries=int(kompas_cfg["open_retries"]),
                    retry_delay_sec=float(kompas_cfg["retry_delay_sec"]),
                )
                try:
                    box = read_gabarit(doc)
                finally:
                    close_document(doc)
                if box:
                    gabarits["body_assembly"] = {
                        **box,
                        "sha256": body.sha256,
                    }

    model = build_kinematics_model(
        config=config,
        config_path=config_path,
        links=links,
        body=body,
        gabarits=gabarits or None,
    )

    export_dir.mkdir(parents=True, exist_ok=True)
    if write_json:
        json_path = write_kinematics_json(
            model,
            export_dir / "kinematics_996.json",
        )
        outputs["json"] = json_path
        logger.info("wrote_json path=%s", json_path)
    if write_header:
        header_path = write_kinematics_header(
            model,
            export_dir / "generated_kinematics_996.h",
        )
        outputs["header"] = header_path
        # Also keep legacy filename as a copy for older docs
        legacy = write_kinematics_header(
            model,
            export_dir / "generated_link_lengths.h",
        )
        outputs["header_legacy"] = legacy
        logger.info("wrote_header path=%s", header_path)
    return outputs
