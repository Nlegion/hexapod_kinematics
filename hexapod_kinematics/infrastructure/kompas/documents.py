"""Open KOMPAS 3D documents with retries."""

from __future__ import annotations

import logging
import time
from pathlib import Path
from typing import Any

from hexapod_kinematics.infrastructure.kompas.session import KompasError, KompasSession

logger = logging.getLogger(__name__)


def open_document(
    session: KompasSession,
    path: Path,
    *,
    open_retries: int = 2,
    retry_delay_sec: float = 1.0,
) -> Any:
    path = path.resolve()
    if not path.is_file():
        raise KompasError(f"CAD file not found: {path}")

    last_error: Exception | None = None
    attempts = open_retries + 1
    for attempt in range(1, attempts + 1):
        try:
            documents = session.app.Documents
            # API7: Documents.Open(path) or OpenEx
            try:
                doc = documents.Open(str(path))
            except Exception:
                doc = documents.OpenEx(str(path), False)
            if doc is None:
                raise KompasError(f"Documents.Open returned None for {path}")
            logger.info("document_opened path=%s", path)
            return doc
        except Exception as exc:  # noqa: BLE001
            last_error = exc
            logger.warning(
                "document_open_failed path=%s attempt=%s/%s error=%s",
                path,
                attempt,
                attempts,
                exc,
            )
            if attempt < attempts:
                time.sleep(retry_delay_sec)
    raise KompasError(
        f"failed to open {path} after {attempts} attempts: {last_error}"
    ) from last_error


def close_document(doc: Any) -> None:
    if doc is None:
        return
    try:
        doc.Close(False)
    except Exception as exc:  # noqa: BLE001
        logger.warning("document_close_failed: %s", exc)
