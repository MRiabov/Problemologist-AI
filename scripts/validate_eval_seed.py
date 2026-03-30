import argparse
import asyncio
import json
import os
import re
import subprocess
import sys
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from evals.logic.stack_profiles import apply_stack_profile_env  # noqa: E402

_STACK_PROFILE_NAME = (
    "integration"
    if os.getenv("IS_INTEGRATION_TEST", "").strip().lower() == "true"
    else "eval"
)
apply_stack_profile_env(_STACK_PROFILE_NAME, env=os.environ, root=ROOT)

from controller.clients.worker import WorkerClient  # noqa: E402
from evals.logic.curation import load_dataset_curation_manifest  # noqa: E402
from evals.logic.models import EvalDatasetItem  # noqa: E402
from evals.logic.specs import AGENT_SPECS  # noqa: E402
from evals.logic.workspace import (  # noqa: E402
    preflight_seeded_entry_contract as _preflight_seeded_entry_contract,
)
from evals.logic.workspace import (  # noqa: E402
    seed_eval_workspace as _seed_eval_workspace,
)
from shared.enums import AgentName  # noqa: E402
from shared.logging import get_logger  # noqa: E402

WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")
logger = get_logger(__name__)


class _QuietLogger:
    def __getattr__(self, _name: str):
        def _noop(*_args, **_kwargs):
            return None

        return _noop


QUIET_LOGGER = _QuietLogger()


async def _wait_for_worker_ready(
    timeout_seconds: float = 60.0,
    poll_interval_seconds: float = 1.0,
    *,
    errors_only: bool = False,
) -> None:
    deadline = time.monotonic() + timeout_seconds
    attempt = 0
    last_error: str | None = None
    temp_client = WorkerClient(base_url=WORKER_LIGHT_URL, session_id="healthcheck")

    while time.monotonic() < deadline:
        attempt += 1
        try:
            health = await temp_client.get_health()
            if health.get("status") == "healthy":
                if not errors_only:
                    logger.info("worker_health_check_ok", attempts=attempt)
                return
            last_error = f"status={health.get('status')}"
        except Exception as exc:
            last_error = str(exc)

        if not errors_only:
            logger.warning(
                "worker_health_check_retry", attempt=attempt, error=last_error
            )
        await asyncio.sleep(poll_interval_seconds)

    raise RuntimeError(
        "Worker did not become healthy within "
        f"{timeout_seconds}s. last_error={last_error}"
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate seeded eval entry contracts without running full evals."
    )
    parser.add_argument(
        "--agent",
        type=str,
        action="append",
        required=True,
        help=(
            "Agent dataset(s) to validate. Supports a single agent, repeated flags, "
            "comma-separated values, list syntax like [a,b], or 'or' separators. "
            "Use 'all' to run every configured agent."
        ),
    )
    parser.add_argument(
        "--task-id",
        type=str,
        default=None,
        help="Validate only one specific task ID.",
    )
    parser.add_argument(
        "--level",
        action="append",
        default=None,
        help=(
            "Validate only evals at the selected complexity levels. Supports a "
            "single level, repeated flags, comma-separated values, or list syntax "
            "like [0,1]."
        ),
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Limit number of rows per agent after task-id filtering.",
    )
    parser.add_argument(
        "--skip-env-up",
        action="store_true",
        help="Skip running scripts/env_up.sh before validation.",
    )
    parser.add_argument(
        "--update-manifests",
        dest="update_manifests",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Repair deterministic seed-manifest drift before validation "
            "(default: enabled)."
        ),
    )
    parser.add_argument(
        "--fail-fast",
        action="store_true",
        help="Stop at the first invalid seed.",
    )
    parser.add_argument(
        "--errors-only",
        action="store_true",
        help=(
            "Suppress PASS/status output and validator success chatter. "
            "Useful for bulk inventory sweeps such as --agent all."
        ),
    )
    parser.add_argument(
        "--concurrency",
        type=int,
        default=16,
        help=(
            "Number of concurrent row validations. Ignored when --fail-fast is enabled."
        ),
    )
    return parser.parse_args()


def _parse_agent_filters(raw_agents: list[str]) -> list[str]:
    parsed: list[str] = []
    for raw in raw_agents:
        token = raw.strip()
        if not token:
            continue
        if token.startswith("[") and token.endswith("]"):
            token = token[1:-1]
        parts = re.split(r"\s*(?:,|\bor\b|\|)\s*", token, flags=re.IGNORECASE)
        parsed.extend(part.strip() for part in parts if part.strip())
    return parsed


def _parse_level_filters(raw_levels: list[str]) -> set[int]:
    parsed: set[int] = set()
    for raw in raw_levels:
        token = raw.strip()
        if not token:
            continue
        if token.startswith("[") and token.endswith("]"):
            token = token[1:-1]
        parts = re.split(r"\s*(?:,|\bor\b|\|)\s*", token, flags=re.IGNORECASE)
        for part in parts:
            value = part.strip()
            if not value:
                continue
            try:
                level = int(value)
            except ValueError as exc:
                raise SystemExit(f"Invalid complexity level '{value}'.") from exc
            if level < 0 or level > 5:
                raise SystemExit(
                    f"Invalid complexity level '{level}'. Expected an integer between 0 and 5."
                )
            parsed.add(level)
    return parsed


def _resolve_agents(agent_args: list[str]) -> list[AgentName]:
    parsed = _parse_agent_filters(agent_args)
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


def _load_dataset(
    agent: AgentName,
    *,
    task_id: str | None,
    limit: int,
    levels: set[int] | None,
) -> list[EvalDatasetItem]:
    dataset_roots = [
        ROOT / "dataset" / "evals" / "datasets",
        ROOT / "dataset" / "data" / "seed" / "role_based",
    ]
    json_path = next(
        (
            root / f"{agent.value}.json"
            for root in dataset_roots
            if (root / f"{agent.value}.json").exists()
        ),
        None,
    )
    if json_path is None:
        searched = ", ".join(str(path) for path in dataset_roots)
        raise FileNotFoundError(
            f"Dataset for agent '{agent.value}' not found. Searched: {searched}"
        )

    with json_path.open() as handle:
        data = json.load(handle)

    if task_id:
        data = [item for item in data if item["id"] == task_id]
    if levels:
        data = [item for item in data if item.get("complexity_level") in levels]
    if limit > 0:
        data = data[:limit]

    seed_dataset = json_path.relative_to(ROOT)
    return [
        EvalDatasetItem.model_validate({**item_raw, "seed_dataset": seed_dataset})
        for item_raw in data
    ]


def _build_session_id(agent: AgentName, task_id: str) -> str:
    suffix = uuid.uuid4().hex[:8]
    return f"seed-check-{agent.value}-{task_id}-{suffix}"[:120]


def _validate_generated_curation_manifests(*, errors_only: bool = False) -> list[Path]:
    manifest_paths = sorted(ROOT.glob("dataset/data/generated/*/v0.0.1/manifest.json"))
    validated: list[Path] = []
    for manifest_path in manifest_paths:
        manifest = load_dataset_curation_manifest(manifest_path)
        if not errors_only:
            logger.info(
                "generated_curation_manifest_validated",
                path=str(manifest_path),
                family=manifest.family,
                agent_target=manifest.agent_target,
                accepted=manifest.counts.accepted_after_pending_filter,
                rejected=manifest.counts.rejected,
            )
        validated.append(manifest_path)
    return validated


async def _validate_item(
    agent: AgentName,
    item: EvalDatasetItem,
    *,
    update_manifests: bool,
    errors_only: bool = False,
) -> tuple[bool, str]:
    session_id = _build_session_id(agent, item.id)
    spec = AGENT_SPECS[agent]
    item_logger = QUIET_LOGGER if errors_only else logger
    try:
        await _seed_eval_workspace(
            item=item,
            session_id=session_id,
            agent_name=agent,
            root=ROOT,
            worker_light_url=WORKER_LIGHT_URL,
            logger=item_logger,
            update_manifests=update_manifests,
        )
        await _preflight_seeded_entry_contract(
            item=item,
            session_id=session_id,
            agent_name=agent,
            spec=spec,
            root=ROOT,
            worker_light_url=WORKER_LIGHT_URL,
            logger=item_logger,
        )
    except Exception as exc:
        return False, str(exc)

    if item.seed_artifact_dir is None and not item.seed_files:
        return True, "prompt-only row; no seeded-entry contract required"
    return True, "seeded-entry contract valid"


def _run_env_up() -> None:
    env_up_path = ROOT / "scripts" / "env_up.sh"
    result = subprocess.run(
        [str(env_up_path), "--profile", "eval"],
        check=True,
        capture_output=True,
        text=True,
    )
    if result.stdout.strip():
        print(result.stdout.strip())
    if result.stderr.strip():
        print(result.stderr.strip(), file=sys.stderr)


async def _async_main(args: argparse.Namespace) -> int:
    if args.concurrency < 1:
        raise SystemExit("--concurrency must be >= 1")

    agents = _resolve_agents(args.agent)
    levels = _parse_level_filters(args.level or [])
    if args.level and not levels:
        raise SystemExit("No valid --level values were parsed.")

    try:
        _validate_generated_curation_manifests(errors_only=args.errors_only)
    except Exception as exc:
        print("Generated curation manifest validation failed.", file=sys.stderr)
        print(str(exc), file=sys.stderr)
        return 1

    if not args.skip_env_up:
        _run_env_up()

    try:
        await _wait_for_worker_ready(errors_only=args.errors_only)
    except Exception as exc:
        print(
            "Worker is not healthy. Start the local services first if needed "
            "(for example `uv run dataset/evals/run_evals.py ...` or `./scripts/env_up.sh`).",
            file=sys.stderr,
        )
        print(str(exc), file=sys.stderr)
        return 1

    failures: list[tuple[str, str, str]] = []
    checked = 0

    work_items: list[tuple[AgentName, EvalDatasetItem]] = []

    for agent in agents:
        dataset = _load_dataset(
            agent,
            task_id=args.task_id,
            limit=args.limit,
            levels=levels if levels else None,
        )
        if args.task_id and not dataset:
            failures.append((agent.value, args.task_id, "task id not found in dataset"))
            if args.fail_fast:
                break
            continue
        work_items.extend((agent, item) for item in dataset)

    if not failures or not args.fail_fast:
        if args.fail_fast or args.concurrency == 1:
            for agent, item in work_items:
                checked += 1
                ok, detail = await _validate_item(
                    agent,
                    item,
                    update_manifests=args.update_manifests,
                    errors_only=args.errors_only,
                )
                if not ok:
                    print(f"FAIL {agent.value} {item.id}: {detail}")
                    failures.append((agent.value, item.id, detail))
                    if args.fail_fast:
                        break
                elif not args.errors_only:
                    print(f"PASS {agent.value} {item.id}: {detail}")
        else:
            semaphore = asyncio.Semaphore(args.concurrency)

            async def _validate_with_semaphore(
                agent: AgentName, item: EvalDatasetItem
            ) -> tuple[AgentName, EvalDatasetItem, bool, str]:
                async with semaphore:
                    ok, detail = await _validate_item(
                        agent,
                        item,
                        update_manifests=args.update_manifests,
                        errors_only=args.errors_only,
                    )
                    return agent, item, ok, detail

            tasks = [
                asyncio.create_task(_validate_with_semaphore(agent, item))
                for agent, item in work_items
            ]
            for completed in asyncio.as_completed(tasks):
                agent, item, ok, detail = await completed
                checked += 1
                if not ok:
                    print(f"FAIL {agent.value} {item.id}: {detail}")
                    failures.append((agent.value, item.id, detail))
                elif not args.errors_only:
                    print(f"PASS {agent.value} {item.id}: {detail}")

    if checked == 0:
        print("No dataset rows matched the requested filter.", file=sys.stderr)
        return 1

    if failures:
        if not args.errors_only:
            print(
                f"Validated {checked} row(s): {checked - len(failures)} passed, {len(failures)} failed.",
                file=sys.stderr,
            )
        return 1

    if not args.errors_only:
        print(f"Validated {checked} row(s): all passed.")
    return 0


def main() -> int:
    args = _parse_args()
    return asyncio.run(_async_main(args))


if __name__ == "__main__":
    raise SystemExit(main())
