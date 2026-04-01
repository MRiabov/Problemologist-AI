from __future__ import annotations

import os
from collections.abc import Mapping, MutableMapping
from enum import StrEnum

from pydantic import BaseModel, Field

from shared.simulation.schemas import (
    SimulatorBackendType,
    get_default_simulator_backend,
)

_DISPLAY_ENV_KEYS = ("DISPLAY", "XAUTHORITY", "WAYLAND_DISPLAY")
_PHYSICS_BACKEND_ENV = "PROBLEMOLOGIST_PHYSICS_GL_BACKEND"
_LEGACY_PHYSICS_BACKEND_ENV = "PROBLEMOLOGIST_MUJOCO_GL_BACKEND"
_RENDER_BACKEND_ENV = "PROBLEMOLOGIST_RENDER_GL_BACKEND"


class HeadlessGLBackend(StrEnum):
    EGL = "egl"
    OSMESA = "osmesa"

    @property
    def vtk_window_class(self) -> str:
        if self is HeadlessGLBackend.EGL:
            return "vtkEGLRenderWindow"
        return "vtkOSOpenGLRenderWindow"


def _normalize_backend(raw_value: object | None) -> HeadlessGLBackend | None:
    if raw_value is None:
        return None

    value = str(raw_value).strip().lower()
    if not value:
        return None

    if value in {"egl", "vtkeglrenderwindow"}:
        return HeadlessGLBackend.EGL
    if value in {"osmesa", "vtkosopenglrenderwindow", "vtkrenderwindow"}:
        return HeadlessGLBackend.OSMESA
    raise ValueError(f"Unsupported headless OpenGL backend: {raw_value!r}")


def _clear_display_env(env: MutableMapping[str, str]) -> None:
    for key in _DISPLAY_ENV_KEYS:
        env.pop(key, None)


def _resolve_physics_backend(
    backend_type: SimulatorBackendType, environ: Mapping[str, str]
) -> HeadlessGLBackend:
    backend = _normalize_backend(
        environ.get(_PHYSICS_BACKEND_ENV, environ.get(_LEGACY_PHYSICS_BACKEND_ENV))
    )
    if backend is not None:
        return backend
    return backend_type.default_headless_gl_backend()


def _resolve_render_backend(environ: Mapping[str, str]) -> HeadlessGLBackend:
    backend = _normalize_backend(environ.get(_RENDER_BACKEND_ENV))
    if backend is not None:
        return backend
    return HeadlessGLBackend.OSMESA


def _resolve_simulation_backend(environ: Mapping[str, str]) -> SimulatorBackendType:
    raw_value = str(environ.get("SIMULATION_DEFAULT_BACKEND", "")).strip().upper()
    if not raw_value:
        return SimulatorBackendType.MUJOCO
    return SimulatorBackendType(raw_value)


class HeadlessOpenGLConfig(BaseModel):
    physics_backend: SimulatorBackendType = Field(
        default_factory=get_default_simulator_backend
    )
    physics_gl_backend: HeadlessGLBackend = Field(default=HeadlessGLBackend.EGL)
    rendering: HeadlessGLBackend = Field(default=HeadlessGLBackend.OSMESA)
    physics_gl_backends_accepted: tuple[HeadlessGLBackend, ...] = Field(
        default_factory=tuple
    )
    render_gl_backends_accepted: tuple[HeadlessGLBackend, ...] = Field(
        default_factory=tuple
    )

    model_config = {"extra": "forbid"}

    @classmethod
    def from_environment(
        cls, environ: Mapping[str, str] | None = None
    ) -> "HeadlessOpenGLConfig":
        env = os.environ if environ is None else environ
        backend_type = _resolve_simulation_backend(env)
        physics_gl_backend = _resolve_physics_backend(backend_type, env)
        rendering_backend = _resolve_render_backend(env)
        return cls.model_validate(
            {
                "physics_backend": backend_type,
                "physics_gl_backend": physics_gl_backend,
                "rendering": rendering_backend,
                "physics_gl_backends_accepted": backend_type.accepted_headless_gl_backends(),
                "render_gl_backends_accepted": (
                    HeadlessGLBackend.EGL,
                    HeadlessGLBackend.OSMESA,
                ),
            }
        )

    @property
    def uses_software_backend(self) -> bool:
        return (
            self.physics_gl_backend is HeadlessGLBackend.OSMESA
            or self.rendering is HeadlessGLBackend.OSMESA
        )

    def model_post_init(self, __context: object) -> None:
        if not self.physics_gl_backends_accepted:
            object.__setattr__(
                self,
                "physics_gl_backends_accepted",
                self.physics_backend.accepted_headless_gl_backends(),
            )
        if not self.render_gl_backends_accepted:
            object.__setattr__(
                self,
                "render_gl_backends_accepted",
                (HeadlessGLBackend.EGL, HeadlessGLBackend.OSMESA),
            )
        if self.physics_gl_backend not in self.physics_gl_backends_accepted:
            accepted = ", ".join(
                backend.value for backend in self.physics_gl_backends_accepted
            )
            raise ValueError(
                f"{self.physics_backend.value} accepts physics GL backends: {accepted}"
            )
        if self.rendering not in self.render_gl_backends_accepted:
            accepted = ", ".join(
                backend.value for backend in self.render_gl_backends_accepted
            )
            raise ValueError(f"renderer accepts render GL backends: {accepted}")

    def apply_physics(self, env: MutableMapping[str, str] | None = None) -> None:
        target = os.environ if env is None else env
        _clear_display_env(target)
        target[_PHYSICS_BACKEND_ENV] = self.physics_gl_backend.value
        target[_LEGACY_PHYSICS_BACKEND_ENV] = self.physics_gl_backend.value
        target["MUJOCO_GL"] = self.physics_gl_backend.value
        if self.physics_gl_backend is HeadlessGLBackend.OSMESA:
            target["LIBGL_ALWAYS_SOFTWARE"] = "1"
        target["PYGLET_HEADLESS"] = "1"

    def apply_rendering(self, env: MutableMapping[str, str] | None = None) -> None:
        target = os.environ if env is None else env
        _clear_display_env(target)
        target[_RENDER_BACKEND_ENV] = self.rendering.value
        target["PYOPENGL_PLATFORM"] = self.rendering.value
        target["VTK_DEFAULT_OPENGL_WINDOW"] = self.rendering.vtk_window_class
        target["PYVISTA_OFF_SCREEN"] = "true"
        target["PYGLET_HEADLESS"] = "1"
        if self.rendering is HeadlessGLBackend.OSMESA:
            target["LIBGL_ALWAYS_SOFTWARE"] = "1"

    def apply(self, env: MutableMapping[str, str] | None = None) -> None:
        target = os.environ if env is None else env
        _clear_display_env(target)
        target[_PHYSICS_BACKEND_ENV] = self.physics_gl_backend.value
        target[_LEGACY_PHYSICS_BACKEND_ENV] = self.physics_gl_backend.value
        target["MUJOCO_GL"] = self.physics_gl_backend.value
        target[_RENDER_BACKEND_ENV] = self.rendering.value
        target["PYOPENGL_PLATFORM"] = self.rendering.value
        target["VTK_DEFAULT_OPENGL_WINDOW"] = self.rendering.vtk_window_class
        target["PYVISTA_OFF_SCREEN"] = "true"
        target["PYGLET_HEADLESS"] = "1"
        if self.uses_software_backend:
            target["LIBGL_ALWAYS_SOFTWARE"] = "1"
        else:
            target.pop("LIBGL_ALWAYS_SOFTWARE", None)


def load_headless_opengl_config(
    environ: Mapping[str, str] | None = None,
) -> HeadlessOpenGLConfig:
    return HeadlessOpenGLConfig.from_environment(environ=environ)


def configure_headless_physics(
    env: MutableMapping[str, str] | None = None,
) -> HeadlessOpenGLConfig:
    config = load_headless_opengl_config(env)
    config.apply_physics(env)
    return config


def configure_headless_mujoco(
    env: MutableMapping[str, str] | None = None,
) -> HeadlessOpenGLConfig:
    """Backward-compatible alias for the physics-backend headless bootstrap."""

    return configure_headless_physics(env)


def configure_headless_rendering(
    env: MutableMapping[str, str] | None = None,
) -> HeadlessOpenGLConfig:
    config = load_headless_opengl_config(env)
    config.apply_rendering(env)
    return config


def configure_headless_runtime(
    env: MutableMapping[str, str] | None = None,
) -> HeadlessOpenGLConfig:
    config = load_headless_opengl_config(env)
    config.apply(env)
    return config
