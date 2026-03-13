import json
import shlex
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
CLI_OUTPUT_START_MARKER = "__PROBLEMOLOGIST_OUTPUT_START__"
CLI_OUTPUT_END_MARKER = "__PROBLEMOLOGIST_OUTPUT_END__"


class CliBackendBootstrapError(RuntimeError):
    """Raised when the external CLI is not usable non-interactively."""


def classify_cli_bootstrap_failure(
    stdout: str | None, stderr: str | None
) -> str | None:
    transcript = "\n".join(part for part in (stdout or "", stderr or "") if part)
    normalized = transcript.lower()

    if (
        "opening authentication page in your browser" in normalized
        or "do you want to continue? [y/n]" in normalized
        or "do you want to continue? [y/n]:" in normalized
    ):
        return (
            "CLI backend requires interactive authentication before it can run "
            "non-interactively. Authenticate the CLI in this environment first, "
            "then rerun the eval."
        )

    if "error authenticating:" in normalized or "loadcodeassist failed" in normalized:
        return (
            "CLI backend authentication/bootstrap failed before model execution. "
            "Ensure the CLI can authenticate and reach its provider APIs, then rerun."
        )

    return None


def is_cli_model(model_name: str | None) -> bool:
    normalized = (model_name or "").strip().lower()
    return normalized == "cli" or normalized.startswith("cli/")


def _cli_alias(model_name: str | None) -> str:
    normalized = (model_name or "").strip()
    if "/" not in normalized:
        return normalized.lower()
    _, alias = normalized.split("/", 1)
    return alias.strip().lower()


def resolve_cli_command_template(
    *,
    model_name: str | None,
    explicit_template: str | None,
) -> str:
    template = (explicit_template or "").strip()
    if template:
        return template

    alias = _cli_alias(model_name)
    if alias.startswith("gemini"):
        return 'gemini --yolo --prompt "$(cat {prompt_file})"'
    if alias.startswith("claude"):
        return 'claude -p "$(cat {prompt_file})"'

    msg = (
        "CLI model selected but no command template is configured. "
        "Set LLM_CLI_COMMAND_TEMPLATE or use a known alias such as cli/gemini."
    )
    raise ValueError(msg)


@dataclass(frozen=True)
class CliCommandRender:
    command: str
    prompt_file: str | None = None


class CliAgentLM:
    """Minimal LM wrapper for external CLI-backed agent/model execution."""

    def __init__(
        self,
        *,
        model_name: str,
        command_template: str,
        timeout_seconds: int,
    ) -> None:
        self.model = model_name
        self.provider = "cli"
        self.command_template = command_template
        self.timeout_seconds = timeout_seconds
        self.history: list[dict[str, Any]] = []
        self.node_type: Any = None

    def render_command(
        self,
        *,
        prompt_text: str,
        prompt_file: str | None,
        session_id: str | None = None,
        node_type: str | None = None,
        repo_root: str | None = None,
    ) -> CliCommandRender:
        substitutions = {
            "prompt": shlex.quote(prompt_text),
            "prompt_json": shlex.quote(json.dumps(prompt_text, ensure_ascii=True)),
            "prompt_file": shlex.quote(prompt_file or ""),
            "session_id": shlex.quote(session_id or ""),
            "node_type": shlex.quote(node_type or ""),
            "repo_root": shlex.quote(repo_root or str(REPO_ROOT)),
        }
        return CliCommandRender(
            command=self.command_template.format(**substitutions),
            prompt_file=prompt_file,
        )

    def __call__(
        self,
        prompt: str | None = None,
        messages: list[dict[str, Any]] | None = None,
        **_kwargs: Any,
    ) -> list[str]:
        prompt_text = str(prompt or "")
        if messages:
            prompt_text += "\n\n".join(str(msg.get("content", "")) for msg in messages)

        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            suffix=".md",
            delete=False,
        ) as handle:
            handle.write(prompt_text)
            prompt_path = handle.name

        try:
            rendered = self.render_command(
                prompt_text=prompt_text,
                prompt_file=prompt_path,
                session_id=None,
                node_type=str(getattr(self.node_type, "value", self.node_type) or ""),
                repo_root=str(REPO_ROOT),
            )
            completed = subprocess.run(
                rendered.command,
                shell=True,
                executable="/bin/bash",
                cwd=str(REPO_ROOT),
                capture_output=True,
                text=True,
                timeout=self.timeout_seconds,
            )
            output_text = (completed.stdout or "").strip() or (
                completed.stderr or ""
            ).strip()
            if completed.returncode != 0:
                msg = (completed.stderr or completed.stdout or "").strip()
                raise RuntimeError(
                    msg or f"CLI command failed with exit code {completed.returncode}"
                )

            self.history.append(
                {
                    "model": self.model,
                    "response_model": self.model,
                    "outputs": [output_text],
                    "response": {
                        "stdout": completed.stdout,
                        "stderr": completed.stderr,
                        "exit_code": completed.returncode,
                    },
                    "usage": None,
                    "cost": None,
                }
            )
            return [output_text]
        finally:
            Path(prompt_path).unlink(missing_ok=True)
