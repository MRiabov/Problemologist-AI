from __future__ import annotations

from shared.runtime.headless import configure_headless_physics


def ensure_headless_physics() -> None:
    """Default the active physics backend to the configured headless GL backend."""

    configure_headless_physics()


def ensure_headless_mujoco() -> None:
    """Backward-compatible alias for the physics headless bootstrap."""

    ensure_headless_physics()
