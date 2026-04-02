from __future__ import annotations

import importlib
from typing import Any

__all__ = [
    "list_render_bundles",
    "pick_preview_pixel",
    "query_render_bundle",
]

_LAZY_ATTRS: dict[str, tuple[str, str]] = {
    "list_render_bundles": ("worker_light.utils.render_query", "list_render_bundles"),
    "pick_preview_pixel": ("worker_light.utils.render_query", "pick_preview_pixel"),
    "query_render_bundle": ("worker_light.utils.render_query", "query_render_bundle"),
}


def __getattr__(name: str) -> Any:
    target = _LAZY_ATTRS.get(name)
    if target is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

    module_name, attr_name = target
    module = importlib.import_module(module_name)
    value = getattr(module, attr_name)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__) | set(_LAZY_ATTRS))
