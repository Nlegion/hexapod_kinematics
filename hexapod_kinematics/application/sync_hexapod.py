"""Sync CAD lengths into hexapod Config.h and report placeholder vs CAD."""

from __future__ import annotations

import logging
import re
from pathlib import Path
from typing import Any

from hexapod_kinematics.domain.kinematics import LinkLengths

logger = logging.getLogger(__name__)

PLACEHOLDER = {"coxa": 50.0, "femur": 80.0, "tibia": 120.0}

DEPENDENT_SCAN_PATTERNS = (
    r"\b50\.0f\b",
    r"\b80\.0f\b",
    r"\b120\.0f\b",
    r"COXA_LENGTH",
    r"FEMUR_LENGTH",
    r"TIBIA_LENGTH",
)


def report_config_diff(
    lengths: LinkLengths | dict[str, float], model: dict[str, Any]
) -> dict[str, Any]:
    L = model.get("lengths_mm") or {}
    if L:
        cad = {
            "coxa": float(L.get("coxa", 0.0)),
            "femur": float(L.get("femur", 0.0)),
            "tibia": float(L.get("tibia", 0.0)),
        }
        tibia_eff = float(L.get("tibia_effective", cad["tibia"]))
    elif isinstance(lengths, LinkLengths):
        cad = {
            "coxa": float(lengths.coxa),
            "femur": float(lengths.femur),
            "tibia": float(lengths.tibia),
        }
        tibia_eff = cad["tibia"]
    else:
        cad = {k: float(lengths[k]) for k in ("coxa", "femur", "tibia")}
        tibia_eff = cad["tibia"]
    pad = (model.get("foot_pad") or {}).get("protrusion_mm")
    return {
        "placeholder_mm": dict(PLACEHOLDER),
        "cad_mm": cad,
        "tibia_effective_mm": tibia_eff,
        "foot_pad_protrusion_mm": pad,
        "delta_mm": {
            "coxa": cad["coxa"] - PLACEHOLDER["coxa"],
            "femur": cad["femur"] - PLACEHOLDER["femur"],
            "tibia": cad["tibia"] - PLACEHOLDER["tibia"],
        },
    }


def scan_dependent_constants(config_h: Path) -> list[str]:
    """Warn if Config.h / nearby still embeds old 50/80/120 literals."""
    warnings: list[str] = []
    if not config_h.is_file():
        return [f"Config.h not found: {config_h}"]
    text = config_h.read_text(encoding="utf-8", errors="replace")
    for pat in (r"\b50\.0f\b", r"\b80\.0f\b", r"\b120\.0f\b"):
        for m in re.finditer(pat, text):
            line_no = text.count("\n", 0, m.start()) + 1
            line = text.splitlines()[line_no - 1].strip()
            if "COXA_LENGTH" in line or "FEMUR_LENGTH" in line or "TIBIA_LENGTH" in line:
                continue  # length defs themselves
            if "TIBIA_EFFECTIVE" in line or "FOOT_PAD" in line:
                continue
            warnings.append(
                f"{config_h.name}:{line_no}: leftover literal matching old length? {line}"
            )
    return warnings


def patch_config_h(
    config_h: Path,
    *,
    coxa: float,
    femur: float,
    tibia: float,
    tibia_effective: float,
    foot_pad_protrusion: float = 2.0,
) -> list[str]:
    """
    Replace COXA/FEMUR/TIBIA_LENGTH and insert TIBIA_EFFECTIVE / FOOT_PAD.
    Returns list of warning strings from dependent-const scan after patch.
    """
    text = config_h.read_text(encoding="utf-8", errors="replace")
    old = text

    def repl_len(name: str, value: float, src: str) -> str:
        pat = rf"constexpr float {name}\s*=\s*[^;]+;"
        repl = f"constexpr float {name} = {value:.6g}f;  // mm (CAD)"
        if not re.search(pat, src):
            raise ValueError(f"{name} not found in {config_h}")
        return re.sub(pat, repl, src, count=1)

    text = repl_len("COXA_LENGTH", coxa, text)
    text = repl_len("FEMUR_LENGTH", femur, text)
    text = repl_len("TIBIA_LENGTH", tibia, text)

    insert_block = (
        f"constexpr float FOOT_PAD_PROTRUSION_MM = {foot_pad_protrusion:.6g}f;  // mm\n"
        f"constexpr float TIBIA_EFFECTIVE_LENGTH = {tibia_effective:.6g}f;  // mm (tibia + pad)\n"
        "// NOTE: Prefer TIBIA_EFFECTIVE_LENGTH for tip FK/IK; TIBIA_LENGTH is link only.\n"
        "// Source: hexapod_kinematics export/generated_kinematics_996.h\n"
    )

    if "TIBIA_EFFECTIVE_LENGTH" in text:
        text = re.sub(
            r"constexpr float FOOT_PAD_PROTRUSION_MM\s*=\s*[^;]+;",
            f"constexpr float FOOT_PAD_PROTRUSION_MM = {foot_pad_protrusion:.6g}f;  // mm",
            text,
            count=1,
        )
        text = re.sub(
            r"constexpr float TIBIA_EFFECTIVE_LENGTH\s*=\s*[^;]+;",
            f"constexpr float TIBIA_EFFECTIVE_LENGTH = {tibia_effective:.6g}f;  // mm (tibia + pad)",
            text,
            count=1,
        )
    else:
        # Insert after TIBIA_LENGTH line (allow trailing comments)
        text = re.sub(
            r"(constexpr float TIBIA_LENGTH\s*=\s*[^;]+;[^\n]*\n)",
            r"\1" + insert_block,
            text,
            count=1,
        )

    if text != old:
        config_h.write_text(text, encoding="utf-8")
        logger.info("patched %s with CAD lengths", config_h)
    else:
        logger.info("no change for %s", config_h)

    warns = scan_dependent_constants(config_h)
    for w in warns:
        logger.warning("%s", w)
    return warns
