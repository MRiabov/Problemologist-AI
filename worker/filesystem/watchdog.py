import os
from pathlib import Path

import httpx
import structlog
from watchdog.events import FileSystemEventHandler
from watchdog.observers import Observer

from worker.utils.storage import StorageClient

logger = structlog.get_logger(__name__)


class STLWatchdogHandler(FileSystemEventHandler):
    def __init__(self, storage_client: StorageClient, controller_url: str | None):
        self.storage_client = storage_client
        self.controller_url = controller_url

    def on_created(self, event):
        self._process(event)

    def on_modified(self, event):
        self._process(event)

    def _process(self, event):
        if event.is_directory:
            return

        filename = Path(event.src_path).name
        if not filename.lower().endswith(".stl"):
            return

        logger.info("stl_file_detected", path=event.src_path)

        try:
            # Upload to S3
            # We use a timestamped or session-based prefix in a real app,
            # but for now we keep it flat or use 'renders/'
            object_name = f"renders/{filename}"
            url = self.storage_client.upload_file(Path(event.src_path), object_name)

            # Notify Controller
            self._notify_controller(filename, url)
        except Exception as e:
            logger.error("watchdog_process_failed", error=str(e))

    def _notify_controller(self, filename: str, url: str):
        if not self.controller_url:
            logger.warning("controller_url_not_set_skipping_notification")
            return

        try:
            payload = {
                "type": "asset_update",
                "asset_type": "stl",
                "filename": filename,
                "url": url,
            }
            # Using httpx synchronously as watchdog callbacks are synchronous
            with httpx.Client() as client:
                # We assume a standard endpoint exists or will exist
                resp = client.post(
                    f"{self.controller_url}/api/v1/notifications/asset",
                    json=payload,
                    timeout=5.0,
                )
                resp.raise_for_status()
                logger.info("controller_notified", filename=filename)
        except Exception as e:
            logger.error("controller_notification_failed", error=str(e))


def start_watchdog(path: str) -> Observer:
    """Starts the filesystem watchdog."""
    controller_url = os.getenv("CONTROLLER_URL")
    try:
        storage_client = StorageClient()
    except Exception as e:
        logger.error("storage_client_init_failed", error=str(e))
        # If storage fails, we can't do much, but we shouldn't crash the app
        # Returning a dummy observer or raising?
        # For robustness, we might want to retry or just log.
        # Here we'll let the observer start but the handler might fail if client is broken.
        # Actually, StorageClient init might fail if env vars are missing.
        # We should probably return None if S3 is not configured?
        # But for now, let's assume it works or fails hard if intended.
        # Re-raising might prevent app startup.
        return None

    event_handler = STLWatchdogHandler(storage_client, controller_url)
    observer = Observer()
    observer.schedule(event_handler, path, recursive=True)
    try:
        observer.start()
        logger.info("watchdog_started", path=path)
        return observer
    except Exception as e:
        logger.error("watchdog_start_failed", error=str(e))
        return None
