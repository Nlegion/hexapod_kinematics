"""Write kinematics JSON export."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def write_kinematics_json(model: dict[str, Any], path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(model, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
    return path
