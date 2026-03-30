from __future__ import annotations

import base64
import subprocess
import tempfile
from contextlib import suppress
from pathlib import Path


def bundle_directory_base64(root: Path) -> str:
    """Pack a workspace directory into a gzipped tarball encoded as base64."""
    with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tf:
        tar_path = Path(tf.name)
    try:
        subprocess.run(
            ["tar", "-zcf", str(tar_path), "-C", str(root), ".", "--no-same-owner"],
            check=True,
            capture_output=True,
            text=True,
        )
        return base64.b64encode(tar_path.read_bytes()).decode("ascii")
    finally:
        with suppress(Exception):
            if tar_path.exists():
                tar_path.unlink()


def extract_bundle_base64(bundle_base64: str, target_dir: Path) -> None:
    """Extract a base64 gzipped tarball into the target directory."""
    bundle_bytes = base64.b64decode(bundle_base64)
    with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tf:
        tf.write(bundle_bytes)
        tar_path = Path(tf.name)
    try:
        subprocess.run(
            ["tar", "-zxf", str(tar_path), "-C", str(target_dir), "--no-same-owner"],
            check=True,
            capture_output=True,
            text=True,
        )
    finally:
        with suppress(Exception):
            if tar_path.exists():
                tar_path.unlink()
