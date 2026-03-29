from __future__ import annotations

from pathlib import Path

from evals.logic.codex_workspace import build_codex_env, prepare_codex_home


def test_codex_home_uses_per_session_cache_roots(tmp_path: Path) -> None:
    workspace_dir = tmp_path / "workspace"
    workspace_dir.mkdir()

    auth_source = tmp_path / "auth.json"
    auth_source.write_text("{}", encoding="utf-8")

    codex_home_root = tmp_path / "codex-home"
    prepare_codex_home(
        codex_home_root=codex_home_root,
        workspace_dir=workspace_dir,
        source_auth_path=auth_source,
    )

    env = build_codex_env(
        task_id="task-001",
        workspace_dir=workspace_dir,
        codex_home_root=codex_home_root,
        session_id="session-001",
    )

    assert env["HOME"] == str(codex_home_root)
    assert env["CODEX_HOME"] == str(codex_home_root / ".codex")
    assert env["XDG_CACHE_HOME"] == str(codex_home_root / ".cache")
    assert env["XDG_CONFIG_HOME"] == str(codex_home_root / ".config")
    assert env["TMPDIR"] == str(codex_home_root / ".tmp")
    assert env["TEMP"] == str(codex_home_root / ".tmp")
    assert env["TMP"] == str(codex_home_root / ".tmp")

    assert (codex_home_root / ".cache").is_dir()
    assert (codex_home_root / ".config").is_dir()
    assert (codex_home_root / ".tmp").is_dir()


def test_codex_env_defaults_to_headless_rendering(tmp_path: Path, monkeypatch) -> None:
    workspace_dir = tmp_path / "workspace"
    workspace_dir.mkdir()

    auth_source = tmp_path / "auth.json"
    auth_source.write_text("{}", encoding="utf-8")

    codex_home_root = tmp_path / "codex-home"
    prepare_codex_home(
        codex_home_root=codex_home_root,
        workspace_dir=workspace_dir,
        source_auth_path=auth_source,
    )

    monkeypatch.setenv("DISPLAY", ":109")
    monkeypatch.setenv("XAUTHORITY", "/tmp/xauthority")

    env = build_codex_env(
        task_id="task-001",
        workspace_dir=workspace_dir,
        codex_home_root=codex_home_root,
        session_id="session-001",
    )

    assert "DISPLAY" not in env
    assert "XAUTHORITY" not in env
    assert env["LIBGL_ALWAYS_SOFTWARE"] == "1"
    assert env["MUJOCO_GL"] == "osmesa"
    assert env["PYOPENGL_PLATFORM"] == "osmesa"
    assert env["PYVISTA_OFF_SCREEN"] == "true"
    assert env["VTK_DEFAULT_OPENGL_RENDERER"] == "OSMesa"
    assert env["PYGLET_HEADLESS"] == "1"
