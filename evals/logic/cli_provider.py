from __future__ import annotations

import os
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Protocol, runtime_checkable

from evals.logic.stack_profiles import apply_stack_profile_env
from shared.agents.config import load_agents_config
from shared.enums import AgentName
from shared.git_utils import repo_revision
from shared.runtime.headless import load_headless_opengl_config

ROOT = Path(__file__).resolve().parents[2]

ReasoningEffort = Literal["low", "medium", "high", "xhigh"]
ReasoningEffortArg = ReasoningEffort | None | object
REASONING_EFFORT_UNSET: object = object()


@runtime_checkable
class CliProvider(Protocol):
    provider_name: str

    def translate_reasoning_effort(
        self, reasoning_effort: ReasoningEffort | None
    ) -> str | None: ...

    def prepare_home(
        self,
        *,
        codex_home_root: Path,
        workspace_dir: Path,
        source_auth_path: Path | None = None,
        agent_name: AgentName | None = None,
        reasoning_effort: ReasoningEffortArg = REASONING_EFFORT_UNSET,
    ) -> Path: ...

    def build_env(
        self,
        *,
        task_id: str,
        workspace_dir: Path,
        codex_home_root: Path,
        session_id: str | None = None,
        agent_name: AgentName | None = None,
    ) -> dict[str, str]: ...

    def build_exec_command(
        self,
        *,
        workspace_dir: Path,
        yolo: bool,
        resume_session_id: str | None = None,
        output_last_message_path: Path | None = None,
    ) -> list[str]: ...

    def build_ui_command(
        self,
        *,
        workspace_dir: Path,
        prompt_text: str,
        yolo: bool,
    ) -> list[str]: ...

    def build_help_command(self) -> list[str]: ...


def _copy_auth_bundle(source_auth_path: Path, codex_home_dir: Path) -> None:
    if not source_auth_path.exists():
        raise FileNotFoundError(f"Codex auth file not found: {source_auth_path}")
    shutil.copy2(source_auth_path, codex_home_dir / "auth.json")


def _execution_flags(*, yolo: bool) -> list[str]:
    if yolo:
        return ["--dangerously-bypass-approvals-and-sandbox"]
    return ["--full-auto"]


@dataclass(frozen=True, slots=True)
class CodexCliProvider:
    provider_name: str = "codex"
    binary_name: str = "codex"

    def prepare_home(
        self,
        *,
        codex_home_root: Path,
        workspace_dir: Path,
        source_auth_path: Path | None = None,
        agent_name: AgentName | None = None,
        reasoning_effort: ReasoningEffortArg = REASONING_EFFORT_UNSET,
    ) -> Path:
        source_auth_path = source_auth_path or (Path.home() / ".codex" / "auth.json")

        codex_home_dir = codex_home_root / ".codex"
        codex_home_dir.mkdir(parents=True, exist_ok=True)
        (codex_home_root / ".cache").mkdir(parents=True, exist_ok=True)
        (codex_home_root / ".config").mkdir(parents=True, exist_ok=True)
        (codex_home_root / ".tmp").mkdir(parents=True, exist_ok=True)
        _copy_auth_bundle(source_auth_path, codex_home_dir)

        workspace_path = workspace_dir.expanduser().resolve()
        from evals.logic import codex_workspace as codex_workspace_module

        config_loader = getattr(
            codex_workspace_module, "load_agents_config", load_agents_config
        )
        config = (
            config_loader()
            if agent_name is not None or reasoning_effort is not REASONING_EFFORT_UNSET
            else None
        )
        if config is not None and not config.llm.reasoning_effort_enabled:
            resolved_reasoning_effort = None
        elif reasoning_effort is REASONING_EFFORT_UNSET:
            resolved_reasoning_effort = (
                config.get_reasoning_effort(agent_name)
                if config is not None and agent_name is not None
                else "high"
            )
        else:
            resolved_reasoning_effort = reasoning_effort

        resolved_reasoning_effort = self.translate_reasoning_effort(
            resolved_reasoning_effort
        )

        config_path = codex_home_dir / "config.toml"
        config_lines = [
            'model = "gpt-5.4-mini"',
            'approvals_reviewer = "user"',
            "",
            "[features]",
            "use_legacy_landlock = true",
            "",
            f'[projects."{workspace_path.as_posix()}"]',
            'trust_level = "trusted"',
            "respect_gitignore = false",
            "",
        ]
        if resolved_reasoning_effort is not None:
            config_lines.insert(
                1, f'model_reasoning_effort = "{resolved_reasoning_effort}"'
            )
        config_path.write_text("\n".join(config_lines), encoding="utf-8")
        return codex_home_dir

    def translate_reasoning_effort(
        self, reasoning_effort: ReasoningEffort | None
    ) -> str | None:
        return reasoning_effort

    def build_env(
        self,
        *,
        task_id: str,
        workspace_dir: Path,
        codex_home_root: Path,
        session_id: str | None = None,
        agent_name: AgentName | None = None,
    ) -> dict[str, str]:
        env = dict(os.environ)
        apply_stack_profile_env("eval", env=env, root=ROOT)
        env.pop("DISPLAY", None)
        env.pop("XAUTHORITY", None)
        env.pop("WAYLAND_DISPLAY", None)
        env["PROBLEMOLOGIST_PHYSICS_GL_BACKEND"] = "egl"
        env["PROBLEMOLOGIST_RENDER_GL_BACKEND"] = "osmesa"
        headless_config = load_headless_opengl_config(env)
        headless_config.apply_physics(env)
        env["PROBLEMOLOGIST_RENDER_GL_BACKEND"] = "osmesa"
        env["PYOPENGL_PLATFORM"] = "osmesa"
        env["VTK_DEFAULT_OPENGL_WINDOW"] = "vtkOSOpenGLRenderWindow"
        env["PYVISTA_OFF_SCREEN"] = "true"
        env["PYGLET_HEADLESS"] = "1"
        env.pop("LIBGL_ALWAYS_SOFTWARE", None)
        env["HOME"] = str(codex_home_root)
        env["CODEX_HOME"] = str(codex_home_root / ".codex")
        env["XDG_CACHE_HOME"] = str(codex_home_root / ".cache")
        env["XDG_CONFIG_HOME"] = str(codex_home_root / ".config")
        env.setdefault("TMPDIR", str(codex_home_root / ".tmp"))
        env.setdefault("TEMP", str(codex_home_root / ".tmp"))
        env.setdefault("TMP", str(codex_home_root / ".tmp"))
        workspace_path = workspace_dir.expanduser().resolve()

        venv_bin = ROOT / ".venv" / "bin"
        if venv_bin.exists():
            current_path = env.get("PATH", "")
            env["PATH"] = (
                f"{venv_bin}{os.pathsep}{current_path}"
                if current_path
                else str(venv_bin)
            )
            env.setdefault("VIRTUAL_ENV", str(ROOT / ".venv"))
            env["PYTHON_BIN"] = str(venv_bin / "python")
        else:
            env.setdefault("PYTHON_BIN", sys.executable)

        env.setdefault("CONTROLLER_URL", "http://localhost:18000")
        env.setdefault("WORKER_LIGHT_URL", "http://localhost:18001")
        env.setdefault(
            "SESSION_ID", session_id or f"local-codex-{task_id}-{os.getpid()}"
        )
        env.setdefault("IS_HEAVY_WORKER", "1")
        env.setdefault("PROBLEMOLOGIST_SCRIPT_IMPORT_MODE", "0")
        env.setdefault("COTS_DB_PATH", str(ROOT / "parts.db"))
        env.setdefault("PROBLEMOLOGIST_REPO_ROOT", str(ROOT))
        current_revision = repo_revision(ROOT)
        if current_revision:
            env.setdefault("REPO_REVISION", current_revision)
        existing_pythonpath = env.get("PYTHONPATH", "")
        pythonpath_entries = [str(workspace_path), str(ROOT)]
        if existing_pythonpath:
            pythonpath_entries.append(existing_pythonpath)
        env["PYTHONPATH"] = os.pathsep.join(
            entry for entry in pythonpath_entries if entry
        )
        if agent_name is not None:
            env.setdefault("AGENT_NAME", agent_name.value)
        return env

    def build_exec_command(
        self,
        *,
        workspace_dir: Path,
        yolo: bool,
        resume_session_id: str | None = None,
        output_last_message_path: Path | None = None,
    ) -> list[str]:
        cmd = [self.binary_name, "exec", "-C", str(workspace_dir)]
        if resume_session_id is not None:
            cmd.extend(["resume", resume_session_id])
        cmd.extend(_execution_flags(yolo=yolo))
        if output_last_message_path is not None:
            output_last_message_path.parent.mkdir(parents=True, exist_ok=True)
            cmd.extend(["--output-last-message", str(output_last_message_path)])
        cmd.extend(
            [
                "-c",
                "shell_environment_policy.inherit=all",
                "--skip-git-repo-check",
                "-",
            ]
        )
        return cmd

    def build_ui_command(
        self,
        *,
        workspace_dir: Path,
        prompt_text: str,
        yolo: bool,
    ) -> list[str]:
        return [
            self.binary_name,
            *_execution_flags(yolo=yolo),
            "-c",
            "shell_environment_policy.inherit=all",
            "--cd",
            str(workspace_dir),
            "--no-alt-screen",
            prompt_text,
        ]

    def build_help_command(self) -> list[str]:
        return [self.binary_name, "exec", "--help"]


_DEFAULT_PROVIDER = CodexCliProvider()


def get_default_cli_provider() -> CliProvider:
    return _DEFAULT_PROVIDER
