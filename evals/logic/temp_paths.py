from __future__ import annotations

import re
import tempfile
from pathlib import Path

DEFAULT_EVAL_TEMP_ROOT = Path("/tmp/problemologist-evals")
DEFAULT_EVAL_FAMILY_NAME = "eval"
_SAFE_FAMILY_NAME_RE = re.compile(r"[^A-Za-z0-9._-]+")


def normalize_eval_family_name(family_name: str) -> str:
    normalized = _SAFE_FAMILY_NAME_RE.sub("-", family_name.strip())
    normalized = normalized.strip("._-")
    return normalized or DEFAULT_EVAL_FAMILY_NAME


def eval_temp_root(family_name: str = DEFAULT_EVAL_FAMILY_NAME) -> Path:
    return DEFAULT_EVAL_TEMP_ROOT / normalize_eval_family_name(family_name)


def mkdtemp_in_eval_temp_root(*, family_name: str, prefix: str) -> Path:
    root = eval_temp_root(family_name)
    root.mkdir(parents=True, exist_ok=True)
    return Path(tempfile.mkdtemp(prefix=prefix, dir=str(root)))
