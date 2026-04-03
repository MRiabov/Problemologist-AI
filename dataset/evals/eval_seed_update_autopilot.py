#!/usr/bin/env python3
from __future__ import annotations

import argparse
import contextlib
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import textwrap
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
DATASET_ROOT = ROOT / "dataset" / "data" / "seed" / "role_based"
DEFAULT_LOG_ROOT = ROOT / "logs" / "evals" / "eval_seed_update_autopilot"
DEFAULT_MIGRATION_FILE = (
    ROOT
    / "specs"
    / "migrations"
    / "minor"
    / "engineering-planner-technical-drawings-migration.md"
)
DEFAULT_NOTES_PATH = DEFAULT_LOG_ROOT / "investigation" / "migration_notes.md"
DEFAULT_STATE_PATH = DEFAULT_LOG_ROOT / "state.json"
DEFAULT_MODEL = "gpt-5.4"
DEFAULT_REASONING_EFFORT = "xhigh"

if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from evals.logic.cli_args import parse_cli_int_set, parse_cli_list_values  # noqa: E402
from evals.logic.models import EvalDatasetItem  # noqa: E402
from evals.logic.specs import AGENT_SPECS  # noqa: E402
from shared.enums import AgentName  # noqa: E402


@dataclass(frozen=True)
class SeedTarget:
    agent: AgentName
    dataset_path: Path
    item: EvalDatasetItem


def utc_now() -> datetime:
    return datetime.now(UTC)


def timestamp_for_path(moment: datetime | None = None) -> str:
    moment = moment or utc_now()
    return moment.strftime("%Y%m%dT%H%M%SZ")


def safe_slug(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_") or "item"


def _resolve_agents(agent_args: list[str]) -> list[AgentName]:
    parsed = parse_cli_list_values(agent_args)
    if not parsed:
        raise SystemExit("No valid --agent values were parsed.")

    if any(agent_arg.lower() == "all" for agent_arg in parsed):
        return list(AGENT_SPECS.keys())

    agents: list[AgentName] = []
    seen: set[AgentName] = set()
    for agent_arg in parsed:
        try:
            agent = AgentName(agent_arg)
        except ValueError as exc:
            available = ", ".join(sorted(agent.value for agent in AGENT_SPECS))
            raise SystemExit(
                f"Unknown agent '{agent_arg}'. Available: {available}"
            ) from exc

        if agent not in AGENT_SPECS:
            raise SystemExit(f"Agent '{agent.value}' is not configured in AGENT_SPECS.")
        if agent in seen:
            continue
        seen.add(agent)
        agents.append(agent)

    return agents


def _load_dataset(agent: AgentName) -> tuple[Path, list[EvalDatasetItem]]:
    dataset_path = DATASET_ROOT / f"{agent.value}.json"
    if not dataset_path.exists():
        raise FileNotFoundError(
            f"Dataset file not found for agent '{agent.value}': {dataset_path}"
        )

    raw = json.loads(dataset_path.read_text(encoding="utf-8"))
    if not isinstance(raw, list):
        raise RuntimeError(f"Expected a JSON array in {dataset_path}")

    seed_dataset = dataset_path.relative_to(ROOT)
    items = [
        EvalDatasetItem.model_validate({**item_raw, "seed_dataset": seed_dataset})
        for item_raw in raw
    ]
    return dataset_path, items


def _select_targets(
    *,
    agents: list[AgentName],
    task_ids: set[str] | None,
    levels: set[int],
    limit: int,
) -> list[SeedTarget]:
    targets: list[SeedTarget] = []
    for agent in agents:
        dataset_path, items = _load_dataset(agent)
        selected: list[EvalDatasetItem] = []
        for item in items:
            if task_ids and item.id not in task_ids:
                continue
            if levels and item.complexity_level not in levels:
                continue
            selected.append(item)
            if limit > 0 and len(selected) >= limit:
                break

        targets.extend(
            SeedTarget(agent=agent, dataset_path=dataset_path, item=item)
            for item in selected
        )

    return targets


def _load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return raw if isinstance(raw, dict) else {}


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _seed_key(agent: AgentName, task_id: str) -> str:
    return f"{agent.value}/{task_id}"


def _automation_session_key(agent: AgentName, task_id: str) -> str:
    return safe_slug(f"eval-seed-update-{agent.value}-{task_id}")


def _resolve_codex_home_root(
    *,
    task_id: str,
    session_id: str | None = None,
    runtime_root: Path | None = None,
) -> Path:
    runtime_root = (runtime_root or ROOT) / ".codex-runtime"
    session_key = session_id or f"local-codex-{task_id}-{os.getpid()}"
    return runtime_root / "homes" / session_key


def _write_codex_home(
    *,
    codex_home_root: Path,
    workspace_dir: Path,
    reasoning_effort: str,
) -> Path:
    source_auth_path = Path.home() / ".codex" / "auth.json"
    if not source_auth_path.exists():
        raise FileNotFoundError(f"Codex auth file not found: {source_auth_path}")

    codex_home_dir = codex_home_root / ".codex"
    codex_home_dir.mkdir(parents=True, exist_ok=True)
    (codex_home_root / ".cache").mkdir(parents=True, exist_ok=True)
    (codex_home_root / ".config").mkdir(parents=True, exist_ok=True)
    (codex_home_root / ".tmp").mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_auth_path, codex_home_dir / "auth.json")

    workspace_path = workspace_dir.expanduser().resolve()
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
    if reasoning_effort:
        config_lines.insert(1, f'model_reasoning_effort = "{reasoning_effort}"')
    (codex_home_dir / "config.toml").write_text(
        "\n".join(config_lines), encoding="utf-8"
    )
    return codex_home_root


def _build_codex_env(
    *,
    task_id: str,
    workspace_dir: Path,
    codex_home_root: Path,
    session_id: str | None = None,
    agent_name: AgentName | None = None,
) -> dict[str, str]:
    env = dict(os.environ)
    env.pop("DISPLAY", None)
    env.pop("XAUTHORITY", None)
    env.pop("WAYLAND_DISPLAY", None)
    env.setdefault("PROBLEMOLOGIST_PHYSICS_GL_BACKEND", "egl")
    env.setdefault("PROBLEMOLOGIST_RENDER_GL_BACKEND", "osmesa")
    env["HOME"] = str(codex_home_root)
    env["CODEX_HOME"] = str(codex_home_root / ".codex")
    env["XDG_CACHE_HOME"] = str(codex_home_root / ".cache")
    env["XDG_CONFIG_HOME"] = str(codex_home_root / ".config")
    env.setdefault("TMPDIR", str(codex_home_root / ".tmp"))
    env.setdefault("TEMP", str(codex_home_root / ".tmp"))
    env.setdefault("TMP", str(codex_home_root / ".tmp"))

    venv_bin = ROOT / ".venv" / "bin"
    if venv_bin.exists():
        current_path = env.get("PATH", "")
        env["PATH"] = (
            f"{venv_bin}{os.pathsep}{current_path}" if current_path else str(venv_bin)
        )
        env.setdefault("VIRTUAL_ENV", str(ROOT / ".venv"))
        env["PYTHON_BIN"] = str(venv_bin / "python")
    else:
        env.setdefault("PYTHON_BIN", sys.executable)

    env.setdefault("CONTROLLER_URL", "http://localhost:18000")
    env.setdefault("WORKER_LIGHT_URL", "http://localhost:18001")
    env.setdefault("SESSION_ID", session_id or f"local-codex-{task_id}-{os.getpid()}")
    env.setdefault("IS_HEAVY_WORKER", "1")
    env.setdefault("PROBLEMOLOGIST_SCRIPT_IMPORT_MODE", "0")
    env.setdefault("COTS_DB_PATH", str(ROOT / "parts.db"))
    env.setdefault("PROBLEMOLOGIST_REPO_ROOT", str(ROOT))
    existing_pythonpath = env.get("PYTHONPATH", "")
    pythonpath_entries = [str(workspace_dir.expanduser().resolve()), str(ROOT)]
    if existing_pythonpath:
        pythonpath_entries.append(existing_pythonpath)
    env["PYTHONPATH"] = os.pathsep.join(entry for entry in pythonpath_entries if entry)
    if agent_name is not None:
        env.setdefault("AGENT_NAME", agent_name.value)
    return env


def _run_command(
    cmd: list[str],
    *,
    prompt_text: str | None,
    cwd: Path,
    env: dict[str, str],
    log_path: Path,
    output_last_message_path: Path | None = None,
) -> tuple[int, str | None]:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    if output_last_message_path is not None:
        output_last_message_path.parent.mkdir(parents=True, exist_ok=True)

    captured_session_id: str | None = None
    with log_path.open("w", encoding="utf-8") as log_file:
        log_file.write(f"[{utc_now().isoformat()}] command: {shlex.join(cmd)}\n")
        log_file.flush()

        proc = subprocess.Popen(
            cmd,
            cwd=str(cwd),
            env=env,
            stdin=subprocess.PIPE if prompt_text is not None else None,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert proc.stdout is not None

        if prompt_text is not None and proc.stdin is not None:
            proc.stdin.write(prompt_text)
            proc.stdin.close()

        for line in proc.stdout:
            log_file.write(line)
            log_file.flush()
            with contextlib.suppress(Exception):
                record = json.loads(line)
                if (
                    captured_session_id is None
                    and isinstance(record, dict)
                    and record.get("type") == "session_meta"
                ):
                    payload = record.get("payload")
                    if isinstance(payload, dict):
                        session_id = str(payload.get("id") or "").strip()
                        if session_id:
                            captured_session_id = session_id

        return_code = proc.wait()
        log_file.write(f"[{utc_now().isoformat()}] exit code: {return_code}\n")
        log_file.flush()

    if output_last_message_path is not None and output_last_message_path.exists():
        # The CLI writes the last assistant message directly; keep it around for
        # the follow-up turn and for later inspection.
        pass

    return return_code, captured_session_id


def _build_codex_prompt_common() -> str:
    return textwrap.dedent(
        """
        Workspace: current directory.
        Use workspace-relative paths only.
        """
    ).strip()


def _build_investigation_prompt(
    *,
    migration_file: Path,
    validator_file: Path,
    notes_path: Path,
) -> str:
    return (
        _build_codex_prompt_common()
        + "\n\n"
        + textwrap.dedent(
            f"""
            Read {migration_file.relative_to(ROOT)} and {validator_file.relative_to(ROOT)}.

            Build a reusable eval-seed update playbook so we do not have to
            rediscover the migration contract for each of the ~70 seeds.

            Focus on:
            - the exact per-role artifact additions implied by the migration
              and the current seed validator,
            - which roles are prompt-only versus seeded-workspace roles,
            - which filenames each role must gain or preserve,
            - what follow-up validation command should run after each update,
            - and any role-specific caveats that should be cached for reuse.

            Do not edit repository files.
            Write the final answer as concise markdown that can be copied into
            {notes_path.relative_to(ROOT)}.
            """
        ).strip()
        + "\n"
    )


def _build_seed_update_prompt(
    *,
    target: SeedTarget,
    investigation_notes: str,
    prior_validation_output: str | None,
) -> str:
    item_json = json.dumps(
        target.item.model_dump(mode="json"), indent=2, sort_keys=True
    )
    parts = [
        _build_codex_prompt_common(),
        "",
        "This is a follow-up turn in the same Codex session. Resume from the prior context.",
        "",
        f"Target agent: {target.agent.value}",
        f"Target dataset file: {target.dataset_path.relative_to(ROOT)}",
        f"Target seed id: {target.item.id}",
        "",
        "Current row:",
        "```json",
        item_json,
        "```",
        "",
        "Cached migration investigation notes:",
        "```markdown",
        investigation_notes.rstrip(),
        "```",
        "",
        "Task:",
        "Update this specific seed so it reflects the migration contract for this role.",
        "Only change the files and row fields that are necessary for this seed.",
        "If this is a seeded downstream role, update the seeded workspace artifacts.",
        "If this is a prompt-only planner row, keep the change scoped to the row.",
        "",
        "Self-validation:",
        f"Run `uv run scripts/validate_eval_seed.py --agent {target.agent.value} --task-id {target.item.id}`",
        "after making the edits.",
        "Use that validation result to fix the seed before ending the turn.",
    ]
    if prior_validation_output:
        parts.extend(
            [
                "",
                "Previous validation output:",
                "```text",
                prior_validation_output.rstrip(),
                "```",
                "Fix the specific issues reported above.",
            ]
        )
    parts.extend(
        [
            "",
            "Return the exact files changed and the validation outcome.",
        ]
    )
    return "\n".join(parts).strip() + "\n"


def _build_validation_command(
    *,
    target: SeedTarget,
    skip_env_up: bool,
    update_manifests: bool,
    errors_only: bool,
) -> list[str]:
    cmd = [
        "uv",
        "run",
        "scripts/validate_eval_seed.py",
        "--agent",
        target.agent.value,
        "--task-id",
        target.item.id,
        "--limit",
        "1",
    ]
    if skip_env_up:
        cmd.append("--skip-env-up")
    if update_manifests:
        cmd.append("--update-manifests")
    else:
        cmd.append("--no-update-manifests")
    if errors_only:
        cmd.append("--errors-only")
    return cmd


def _run_validation(
    *,
    target: SeedTarget,
    run_dir: Path,
    skip_env_up: bool,
    update_manifests: bool,
    errors_only: bool,
) -> tuple[int, Path]:
    cmd = _build_validation_command(
        target=target,
        skip_env_up=skip_env_up,
        update_manifests=update_manifests,
        errors_only=errors_only,
    )
    log_path = run_dir / "validation.log"
    completed = subprocess.run(
        cmd,
        cwd=str(ROOT),
        env=dict(os.environ),
        text=True,
        capture_output=True,
        check=False,
    )
    log_text = "".join(
        part
        for part in (
            completed.stdout or "",
            ("\n" if completed.stdout and completed.stderr else ""),
            completed.stderr or "",
        )
    )
    log_path.write_text(
        f"[{utc_now().isoformat()}] command: {shlex.join(cmd)}\n{log_text}",
        encoding="utf-8",
    )
    return completed.returncode, log_path


def _load_state(path: Path) -> dict[str, Any]:
    state = _load_json(path)
    if not state:
        return {"version": 1, "investigation": {}, "seeds": {}}
    state.setdefault("version", 1)
    state.setdefault("investigation", {})
    state.setdefault("seeds", {})
    return state


def _save_state(path: Path, state: dict[str, Any]) -> None:
    state["updated_at"] = utc_now().isoformat()
    _write_json(path, state)


def _prepare_codex_run_home(
    *,
    automation_key: str,
    workspace_dir: Path,
    agent_name: AgentName | None,
    reasoning_effort: str,
    log_root: Path,
) -> Path:
    codex_home_root = _resolve_codex_home_root(
        task_id=automation_key,
        session_id=automation_key,
        runtime_root=log_root / "codex-runtime",
    )
    _write_codex_home(
        codex_home_root=codex_home_root,
        workspace_dir=workspace_dir,
        reasoning_effort=reasoning_effort,
    )
    return codex_home_root


def _run_codex_turn(
    *,
    prompt_text: str,
    workspace_dir: Path,
    automation_key: str,
    agent_name: AgentName | None,
    session_id: str | None,
    model: str,
    reasoning_effort: str,
    log_path: Path,
    output_last_message_path: Path,
    log_root: Path,
) -> tuple[int, str | None]:
    codex_home_root = _prepare_codex_run_home(
        automation_key=automation_key,
        workspace_dir=workspace_dir,
        agent_name=agent_name,
        reasoning_effort=reasoning_effort,
        log_root=log_root,
    )
    env = _build_codex_env(
        task_id=automation_key,
        workspace_dir=workspace_dir,
        codex_home_root=codex_home_root,
        session_id=automation_key,
        agent_name=agent_name,
    )

    cmd = ["codex", "exec"]
    if session_id is not None:
        cmd.extend(["resume", session_id])
    cmd.extend(
        [
            "-m",
            model,
            "-c",
            f'model_reasoning_effort="{reasoning_effort}"',
            "--json",
            "--output-last-message",
            str(output_last_message_path),
            "-c",
            "shell_environment_policy.inherit=all",
            "--dangerously-bypass-approvals-and-sandbox",
            "--cd",
            str(workspace_dir),
            "--skip-git-repo-check",
            "-",
        ]
    )

    output_last_message_path.parent.mkdir(parents=True, exist_ok=True)
    return_code, captured_session_id = _run_command(
        cmd,
        prompt_text=prompt_text,
        cwd=workspace_dir,
        env=env,
        log_path=log_path,
        output_last_message_path=output_last_message_path,
    )
    return return_code, captured_session_id


def _print_targets(targets: list[SeedTarget]) -> None:
    for target in targets:
        print(
            f"{target.agent.value} {target.item.id} "
            f"(level {target.item.complexity_level})"
        )


def _investigate(args: argparse.Namespace) -> int:
    log_root = args.log_root
    run_root = log_root / "investigation" / timestamp_for_path()
    run_root.mkdir(parents=True, exist_ok=True)

    prompt_path = run_root / "prompt.md"
    output_last_message_path = run_root / "last_message.md"
    log_path = run_root / "codex.jsonl"

    prompt_text = _build_investigation_prompt(
        migration_file=args.migration_file,
        validator_file=ROOT / "scripts" / "validate_eval_seed.py",
        notes_path=args.notes_path,
    )
    prompt_path.write_text(prompt_text, encoding="utf-8")

    if getattr(args, "dry_run", False):
        print(prompt_path)
        return 0

    rc, session_id = _run_codex_turn(
        prompt_text=prompt_text,
        workspace_dir=ROOT,
        automation_key="migration-investigation",
        agent_name=None,
        session_id=_load_state(args.state_path)
        .get("investigation", {})
        .get("session_id"),
        model=args.model,
        reasoning_effort=args.reasoning_effort,
        log_path=log_path,
        output_last_message_path=output_last_message_path,
        log_root=log_root,
    )

    notes_text = (
        output_last_message_path.read_text(encoding="utf-8")
        if output_last_message_path.exists()
        else ""
    )
    if notes_text.strip():
        args.notes_path.parent.mkdir(parents=True, exist_ok=True)
        args.notes_path.write_text(notes_text, encoding="utf-8")

    state = _load_state(args.state_path)
    state["investigation"] = {
        "session_id": session_id,
        "prompt_path": str(prompt_path),
        "output_last_message_path": str(output_last_message_path),
        "log_path": str(log_path),
        "notes_path": str(args.notes_path),
        "return_code": rc,
    }
    _save_state(args.state_path, state)

    print(f"Investigation prompt: {prompt_path}")
    print(f"Investigation log: {log_path}")
    print(f"Investigation notes: {args.notes_path}")
    if session_id:
        print(f"Codex session: {session_id}")
    return rc


def _apply(args: argparse.Namespace) -> int:
    state = _load_state(args.state_path)
    if not args.notes_path.exists():
        if args.auto_investigate:
            rc = _investigate(args)
            if rc != 0:
                return rc
            state = _load_state(args.state_path)
        else:
            raise SystemExit(
                f"Investigation notes not found: {args.notes_path}. "
                "Run `investigate` first or pass --auto-investigate."
            )

    investigation_notes = args.notes_path.read_text(encoding="utf-8").strip()
    if not investigation_notes:
        raise SystemExit(f"Investigation notes are empty: {args.notes_path}")

    agents = _resolve_agents(args.agent)
    raw_task_id_tokens = [token for group in (args.task_id or []) for token in group]
    task_ids = set(parse_cli_list_values(raw_task_id_tokens))
    if args.task_id and not task_ids:
        raise SystemExit("No valid --task-id values were parsed.")
    levels = parse_cli_int_set(args.level or [])
    if args.level and not levels:
        raise SystemExit("No valid --level values were parsed.")

    targets = _select_targets(
        agents=agents,
        task_ids=task_ids,
        levels=levels,
        limit=args.limit,
    )
    if not targets:
        raise SystemExit("No dataset rows matched the requested filter.")

    if args.list_only:
        _print_targets(targets)
        return 0

    run_root = args.log_root / "runs" / timestamp_for_path()
    run_root.mkdir(parents=True, exist_ok=True)

    failures: list[str] = []
    for target in targets:
        seed_key = _seed_key(target.agent, target.item.id)
        session_key = _automation_session_key(target.agent, target.item.id)
        seed_state = state.setdefault("seeds", {}).get(seed_key, {})
        session_id = seed_state.get("session_id")
        prior_validation_output: str | None = None
        seed_run_root = run_root / safe_slug(seed_key)
        seed_run_root.mkdir(parents=True, exist_ok=True)

        print(f"Updating {seed_key}")
        if session_id:
            print(f"Resuming Codex session {session_id}")

        validated = False
        for attempt in range(1, args.max_rounds + 1):
            prompt_text = _build_seed_update_prompt(
                target=target,
                investigation_notes=investigation_notes,
                prior_validation_output=prior_validation_output,
            )
            prompt_path = seed_run_root / f"prompt-round-{attempt}.md"
            prompt_path.write_text(prompt_text, encoding="utf-8")

            codex_log_path = seed_run_root / f"codex-round-{attempt}.jsonl"
            output_last_message_path = (
                seed_run_root / f"last-message-round-{attempt}.md"
            )
            return_code, captured_session_id = _run_codex_turn(
                prompt_text=prompt_text,
                workspace_dir=ROOT,
                automation_key=session_key,
                agent_name=target.agent,
                session_id=session_id,
                model=args.model,
                reasoning_effort=args.reasoning_effort,
                log_path=codex_log_path,
                output_last_message_path=output_last_message_path,
                log_root=args.log_root,
            )
            if captured_session_id:
                session_id = captured_session_id

            validation_rc, validation_log_path = _run_validation(
                target=target,
                run_dir=seed_run_root / f"validation-round-{attempt}",
                skip_env_up=args.skip_env_up,
                update_manifests=args.update_manifests,
                errors_only=args.errors_only,
            )

            seed_state = {
                "session_id": session_id,
                "session_key": session_key,
                "prompt_path": str(prompt_path),
                "codex_log_path": str(codex_log_path),
                "last_message_path": str(output_last_message_path),
                "validation_log_path": str(validation_log_path),
                "validation_rc": validation_rc,
                "codex_return_code": return_code,
                "attempts": attempt,
            }
            state["seeds"][seed_key] = seed_state
            _save_state(args.state_path, state)

            if return_code != 0:
                print(
                    f"Codex turn exited {return_code} for {seed_key}; validating anyway."
                )

            if validation_rc == 0:
                print(f"Validated {seed_key} on attempt {attempt}.")
                validated = True
                break

            validation_log_text = validation_log_path.read_text(encoding="utf-8")
            prior_validation_output = validation_log_text
            print(
                f"Validation failed for {seed_key} on attempt {attempt}; "
                "resuming the same session."
            )
        if not validated:
            failures.append(seed_key)

    if failures:
        print("Failed seeds:")
        for seed_key in failures:
            print(f"- {seed_key}")
        return 1

    print(f"Updated and validated {len(targets)} seed(s).")
    return 0


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Cache one migration investigation and batch-update eval seeds through "
            "resumable Codex sessions."
        )
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--log-root",
        type=Path,
        default=DEFAULT_LOG_ROOT,
        help="Directory that receives cached notes, prompts, and logs.",
    )
    common.add_argument(
        "--state-path",
        type=Path,
        default=DEFAULT_STATE_PATH,
        help="Path to the persistent session-state JSON file.",
    )
    common.add_argument(
        "--notes-path",
        type=Path,
        default=DEFAULT_NOTES_PATH,
        help="Cached investigation notes file reused by update runs.",
    )
    common.add_argument(
        "--migration-file",
        type=Path,
        default=DEFAULT_MIGRATION_FILE,
        help="Migration file to inspect during the investigation pass.",
    )
    common.add_argument(
        "--model",
        type=str,
        default=DEFAULT_MODEL,
        help="Codex model to use for investigation and seed updates.",
    )
    common.add_argument(
        "--reasoning-effort",
        type=str,
        default=DEFAULT_REASONING_EFFORT,
        choices=["low", "medium", "high", "xhigh"],
        help="Codex reasoning effort to request.",
    )
    common.add_argument(
        "--errors-only",
        action="store_true",
        help="Suppress validation success chatter.",
    )

    investigate = subparsers.add_parser(
        "investigate",
        parents=[common],
        help="Run the one-time migration investigation and cache the notes.",
    )
    investigate.add_argument(
        "--dry-run",
        action="store_true",
        help="Write the investigation prompt without launching Codex.",
    )
    investigate.set_defaults(func=_investigate)

    apply_parser = subparsers.add_parser(
        "apply",
        parents=[common],
        help="Update the selected eval seeds using cached investigation notes.",
    )
    apply_parser.add_argument(
        "--agent",
        type=str,
        action="append",
        required=True,
        help=(
            "Agent dataset(s) to process. Supports a single agent, repeated flags, "
            "comma-separated values, list syntax like [a,b], or 'or' separators. "
            "Use 'all' to run every configured agent."
        ),
    )
    apply_parser.add_argument(
        "--task-id",
        action="append",
        nargs="+",
        default=None,
        help=(
            "Process only the selected task IDs. Supports a single ID, repeated "
            "flags, comma-separated values, or list syntax like [a,b]."
        ),
    )
    apply_parser.add_argument(
        "--level",
        action="append",
        default=None,
        help=(
            "Process only evals at the selected complexity levels. Supports a "
            "single level, repeated flags, comma-separated values, or list syntax "
            "like [0,1]."
        ),
    )
    apply_parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Limit the number of rows per agent after filtering.",
    )
    apply_parser.add_argument(
        "--auto-investigate",
        action="store_true",
        help="Run the investigation pass first if the notes file is missing.",
    )
    apply_parser.add_argument(
        "--list-only",
        action="store_true",
        help="Print the selected seeds without launching Codex.",
    )
    apply_parser.add_argument(
        "--max-rounds",
        type=int,
        default=2,
        help="Maximum Codex/validation rounds per seed.",
    )
    apply_parser.add_argument(
        "--skip-env-up",
        action="store_true",
        help="Pass --skip-env-up to scripts/validate_eval_seed.py.",
    )
    apply_parser.add_argument(
        "--update-manifests",
        dest="update_manifests",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Let the validator repair deterministic manifest drift.",
    )
    apply_parser.set_defaults(func=_apply)

    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    if args.command == "investigate":
        if args.migration_file is None:
            raise SystemExit("--migration-file is required for investigate.")
        if not args.migration_file.exists():
            raise SystemExit(f"Migration file not found: {args.migration_file}")
    if args.command == "apply" and args.max_rounds < 1:
        raise SystemExit("--max-rounds must be >= 1")
    if args.command == "apply" and args.limit < 0:
        raise SystemExit("--limit must be >= 0")
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
