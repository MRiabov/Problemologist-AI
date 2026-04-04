from __future__ import annotations

from io import BytesIO
from pathlib import Path

import structlog
from PIL import Image

from controller.clients.worker import WorkerClient

_IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg"}
_MIN_RENDER_WIDTH = 640
_MIN_RENDER_HEIGHT = 480

logger = structlog.get_logger(__name__)


def _normalize_render_root(render_root: str) -> str:
    normalized = render_root.strip("/")
    return normalized or "renders"


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
    require_images: bool = False,
    min_width: int = _MIN_RENDER_WIDTH,
    min_height: int = _MIN_RENDER_HEIGHT,
) -> str | None:
    """
    Fail closed if seeded or reviewed RGB render images are blank/black.

    This intentionally looks only at render image files under the render root.
    Depth and segmentation outputs are not part of this gate.
    """
    root = _normalize_render_root(render_root)
    if not await worker_client.exists(root):
        if require_images:
            return f"render validation failed: required render root missing: {root}"
        return None

    try:
        image_paths = await _collect_render_image_paths(worker_client, render_root=root)
    except Exception as exc:
        return f"render validation failed while enumerating {root}: {exc}"

    if not image_paths:
        if require_images:
            return f"render validation failed: no render images were found under {root}"
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
            with Image.open(BytesIO(binary)) as image:
                width, height = image.size
                if width < min_width or height < min_height:
                    logger.error(
                        "render_artifact_too_small",
                        render_root=root,
                        path=path,
                        width=width,
                        height=height,
                        min_width=min_width,
                        min_height=min_height,
                    )
                    return (
                        "render artifact too small: "
                        f"{path} is {width}x{height}, minimum is "
                        f"{min_width}x{min_height}"
                    )
                rgb_image = image.convert("RGB")
                if rgb_image.getbbox() is None:
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
