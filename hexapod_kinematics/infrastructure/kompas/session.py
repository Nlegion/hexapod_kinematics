"""KOMPAS COM session: hidden app, retries, guaranteed Quit."""

from __future__ import annotations

import logging
import time
from types import TracebackType
from typing import Any

logger = logging.getLogger(__name__)


class KompasError(RuntimeError):
    """Readable wrapper around COM failures."""


class KompasSession:
    """Context manager for KOMPAS.Application.7 (Visible=False)."""

    def __init__(
        self,
        *,
        progid: str,
        connect_retries: int = 2,
        retry_delay_sec: float = 1.0,
    ) -> None:
        self._progid = progid
        self._connect_retries = connect_retries
        self._retry_delay_sec = retry_delay_sec
        self._app: Any = None
        self._comtypes: Any = None

    @property
    def app(self) -> Any:
        if self._app is None:
            raise KompasError("KOMPAS session is not started")
        return self._app

    def __enter__(self) -> KompasSession:
        self._app = self._create_app()
        try:
            self._app.Visible = False
        except Exception as exc:  # noqa: BLE001 — COM attribute quirks
            logger.warning("could not set Visible=False: %s", exc)
        logger.info("kompas_started progid=%s", self._progid)
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()

    def _create_app(self) -> Any:
        try:
            import comtypes.client as comtypes_client
        except ImportError as exc:
            raise KompasError(
                "comtypes is required to talk to KOMPAS; pip install comtypes"
            ) from exc
        self._comtypes = comtypes_client
        last_error: Exception | None = None
        attempts = self._connect_retries + 1
        for attempt in range(1, attempts + 1):
            try:
                return comtypes_client.CreateObject(self._progid)
            except Exception as exc:  # noqa: BLE001 — COMError variants
                last_error = exc
                logger.warning(
                    "kompas_connect_failed attempt=%s/%s error=%s",
                    attempt,
                    attempts,
                    exc,
                )
                if attempt < attempts:
                    time.sleep(self._retry_delay_sec)
        raise KompasError(
            f"failed to create {self._progid} after {attempts} attempts: {last_error}"
        ) from last_error

    def close(self) -> None:
        app = self._app
        self._app = None
        if app is None:
            return
        try:
            documents = getattr(app, "Documents", None)
            if documents is not None:
                # Close from last to first
                try:
                    count = int(documents.Count)
                except Exception:  # noqa: BLE001
                    count = 0
                # Documents.Item is 0-based in API7
                for index in range(count - 1, -1, -1):
                    try:
                        doc = documents.Item(index)
                        if doc is not None:
                            doc.Close(False)
                    except Exception as exc:  # noqa: BLE001
                        logger.warning("document_close_failed index=%s: %s", index, exc)
        except Exception as exc:  # noqa: BLE001
            logger.warning("documents_cleanup_failed: %s", exc)
        try:
            app.Quit()
            logger.info("kompas_quit")
        except Exception as exc:  # noqa: BLE001
            logger.warning("kompas_quit_failed: %s", exc)
