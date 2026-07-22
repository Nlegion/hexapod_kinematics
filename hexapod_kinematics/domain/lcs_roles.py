"""Resolve LCS names to roles and detect duplicates."""

from __future__ import annotations

import logging
from collections import defaultdict

from hexapod_kinematics.core.constants import lcs as lcs_const
from hexapod_kinematics.domain.frames import Frame3D

logger = logging.getLogger(__name__)


def resolve_role(name: str, role_map: dict[str, str]) -> str | None:
    if name in role_map:
        return role_map[name]
    return None


def index_frames_by_role(
    frames: list[Frame3D],
    role_map: dict[str, str],
) -> tuple[dict[str, Frame3D], list[str]]:
    """
    Map role -> first Frame3D.

    Duplicate names: keep first, mark ambiguous, log warning.
    Unknown names: collected as warnings (not roles).
    """
    by_name: dict[str, list[Frame3D]] = defaultdict(list)
    for frame in frames:
        by_name[frame.name].append(frame)

    role_to_frame: dict[str, Frame3D] = {}
    warnings: list[str] = []

    for name, group in by_name.items():
        chosen = group[0]
        ambiguous = len(group) > 1
        if ambiguous:
            msg = (
                f"duplicate LCS name {name!r}: count={len(group)}; "
                "using first (first_with_warning)"
            )
            logger.warning(msg)
            warnings.append(msg)
            chosen = Frame3D(
                name=chosen.name,
                origin=chosen.origin,
                axis_x=chosen.axis_x,
                axis_y=chosen.axis_y,
                axis_z=chosen.axis_z,
                ambiguous=True,
                synthesized=chosen.synthesized,
            )
        role = resolve_role(name, role_map)
        if role is None:
            continue
        if role in role_to_frame:
            msg = f"role {role!r} already bound; ignoring later LCS {name!r}"
            logger.warning(msg)
            warnings.append(msg)
            continue
        role_to_frame[role] = chosen

    return role_to_frame, warnings


def default_role_map_merged(extra: dict[str, str] | None) -> dict[str, str]:
    merged = dict(lcs_const.DEFAULT_LCS_ROLE_MAP)
    if extra:
        merged.update(extra)
    return merged
