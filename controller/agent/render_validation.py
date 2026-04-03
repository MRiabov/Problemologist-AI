from __future__ import annotations

from io import BytesIO
from pathlib import Path

import structlog
from PIL import Image

from controller.clients.worker import WorkerClient

_IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg"}

logger = structlog.get_logger(__name__)


def _normalize_render_root(render_root: str) -> str:
    normalized = render_root.strip("/")
    return normalized or "renders"


def _is_non_black_image(binary: bytes) -> bool:
    with Image.open(BytesIO(binary)) as image:
        rgb_image = image.convert("RGB")
        return rgb_image.getbbox() is not None


async def _collect_render_image_paths(
    worker_client: WorkerClient, *, render_root: str
) -> list[str]:
    root = _normalize_render_root(render_root)
    pending: list[str] = [root]
    seen_dirs: set[str] = set()
    image_paths: list[str] = []

    while pending:
        current = pending.pop()
        if current in seen_dirs:
            continue
        seen_dirs.add(current)

        entries = await worker_client.list_files(current)
        for entry in entries:
            entry_path = entry.path.lstrip("/")
            if entry.is_dir:
                pending.append(entry_path)
                continue

            if Path(entry.name).suffix.lower() in _IMAGE_EXTENSIONS:
                image_paths.append(entry_path)

    return sorted(image_paths)


async def validate_render_images_non_black(
    worker_client: WorkerClient,
    *,
    render_root: str = "renders",
) -> str | None:
    """
    Fail closed if seeded or reviewed RGB render images are blank/black.

    This intentionally looks only at render image files under the render root.
    Depth and segmentation outputs are not part of this gate.
    """
    root = _normalize_render_root(render_root)
    if not await worker_client.exists(root):
        return None

    try:
        image_paths = await _collect_render_image_paths(worker_client, render_root=root)
    except Exception as exc:
        return f"render validation failed while enumerating {root}: {exc}"

    if not image_paths:
        return None

    binary_map = await worker_client.read_files_binary(image_paths)

    for path in image_paths:
        try:
            binary = binary_map.get(path)
            if binary is None:
                return f"render artifact unreadable: {path}: missing from batch read"
            if not binary:
                logger.error(
                    "render_artifact_empty",
                    render_root=root,
                    path=path,
                )
                return f"render artifact is empty: {path}"
            if not _is_non_black_image(binary):
                logger.error(
                    "render_artifact_black_or_empty",
                    render_root=root,
                    path=path,
                )
                return f"render artifact appears black/empty: {path}"
        except Exception as exc:
            logger.error(
                "render_artifact_unreadable",
                render_root=root,
                path=path,
                error=str(exc),
            )
            return f"render artifact unreadable: {path}: {exc}"

    return None
