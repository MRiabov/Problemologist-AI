from __future__ import annotations

import argparse
import asyncio
import atexit
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from evals.logic.dataset_selection import (  # noqa: E402
    parse_level_filters,
    resolve_agents,
)
from evals.logic.stack_profiles import apply_stack_profile_env  # noqa: E402
from scripts.internal.eval_run_lock import (  # noqa: E402
    EvalRunSelection,
    acquire_eval_run_shared_lock,
    release_eval_run_lock,
)
from scripts.internal.eval_seed_selection import (  # noqa: E402
    infer_seed_agent_for_task_id,
    load_seed_dataset,
)
from shared.agents.config import TECHNICAL_DRAWING_MODE_ENV, DraftingMode  # noqa: E402
from shared.enums import AgentName  # noqa: E402

_STACK_PROFILE_NAME = (
    "integration"
    if os.getenv("IS_INTEGRATION_TEST", "").strip().lower() == "true"
    else "eval"
)
apply_stack_profile_env(_STACK_PROFILE_NAME, env=os.environ, root=ROOT)


def _resolve_seed_artifact_dir(item, *, root: Path) -> Path | None:
    if item.seed_artifact_dir is None:
        return None

    artifact_dir = Path(item.seed_artifact_dir)
    if artifact_dir.is_absolute():
        return artifact_dir

    repo_relative = root / artifact_dir
    if repo_relative.exists():
        return repo_relative

    if item.seed_dataset is not None:
        dataset_relative = (root / item.seed_dataset).parent / artifact_dir
        if dataset_relative.exists():
            return dataset_relative

    return repo_relative


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Regenerate deterministic seeded eval render bundles without "
            "running seed validation."
        )
    )
    parser.add_argument(
        "--agent",
        type=str,
        action="append",
        required=False,
        help=(
            "Agent dataset(s) to update. Supports a single agent, repeated "
            "flags, comma-separated values, list syntax like [a,b], or 'or' "
            "separators. Use 'all' to run every configured agent. Omit when "
            "--task-id is set to infer the matching agent automatically."
        ),
    )
    parser.add_argument(
        "--task-id",
        type=str,
        default=None,
        help="Update renders only for one specific task ID.",
    )
    parser.add_argument(
        "--level",
        action="append",
        default=None,
        help=(
            "Update renders only for the selected complexity levels. Supports "
            "a single level, repeated flags, comma-separated values, or list "
            "syntax like [0,1]."
        ),
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Limit number of rows per agent after task-id filtering.",
    )
    parser.add_argument(
        "--queue",
        action="store_true",
        help=(
            "Wait for the eval lock when another eval consumer is active "
            "instead of failing fast."
        ),
    )
    parser.add_argument(
        "--technical-drawing-mode",
        type=str,
        default=DraftingMode.FULL.value,
        choices=[
            DraftingMode.OFF.value,
            DraftingMode.MINIMAL.value,
            DraftingMode.FULL.value,
        ],
        help=(
            "Select the drawing-mode corpus to update (default: full). "
            "Rows without technical_drawing_mode are skipped."
        ),
    )
    parser.add_argument(
        "--errors-only",
        action="store_true",
        help="Suppress per-row progress output.",
    )
    return parser.parse_args()


def _update_item(
    agent: AgentName,
    item,
    *,
    errors_only: bool,
) -> tuple[bool, bool, str]:
    artifact_dir = _resolve_seed_artifact_dir(item, root=ROOT)
    if artifact_dir is None:
        if not errors_only:
            print(f"SKIP {agent.value} {item.id}: prompt-only row")
        return True, False, "prompt-only row; no seeded artifact directory"

    from scripts.internal.eval_seed_renders import update_seed_artifact_renders

    saved_paths = update_seed_artifact_renders(artifact_dir)
    saved_count = len(saved_paths)
    if not errors_only:
        print(f"UPDATED {agent.value} {item.id}: {saved_count} render file(s)")
    return True, True, "seed render bundle updated"


async def _async_main(args: argparse.Namespace) -> int:
    technical_drawing_mode = DraftingMode(args.technical_drawing_mode)
    if args.agent:
        agents = resolve_agents(args.agent)
    elif args.task_id:
        agents = [infer_seed_agent_for_task_id(args.task_id, root=ROOT)]
    else:
        raise SystemExit("Provide --agent or --task-id.")
    levels = parse_level_filters(args.level)
    if args.level and not levels:
        raise SystemExit("No valid --level values were parsed.")

    lock_lease = acquire_eval_run_shared_lock(
        queue=args.queue,
        requested_command=[sys.argv[0], *sys.argv[1:]],
        requested_selection=EvalRunSelection(
            agent=agents[0].value if len(agents) == 1 else None,
            task_ids=[args.task_id] if args.task_id else [],
            levels=sorted(levels) if levels else [],
            technical_drawing_mode=technical_drawing_mode.value,
        ),
    )
    if lock_lease is None:
        return 1
    atexit.register(release_eval_run_lock, lock_lease)

    checked = 0
    touched = 0
    skipped = 0
    failures: list[tuple[str, str, str]] = []

    for agent in agents:
        dataset = load_seed_dataset(
            agent,
            task_id=args.task_id,
            limit=args.limit,
            levels=levels if levels else None,
            technical_drawing_mode=technical_drawing_mode,
            root=ROOT,
        )
        if args.task_id and not dataset:
            failures.append((agent.value, args.task_id, "task id not found in dataset"))
            continue
        for item in dataset:
            checked += 1
            try:
                ok, rendered, detail = _update_item(
                    agent,
                    item,
                    errors_only=args.errors_only,
                )
            except Exception as exc:
                ok, rendered, detail = False, False, str(exc)
            if not ok:
                failures.append((agent.value, item.id, detail))
            elif rendered:
                touched += 1
            else:
                skipped += 1

    if checked == 0 and not failures:
        print("No dataset rows matched the requested filter.", file=sys.stderr)
        return 1

    if failures:
        if not args.errors_only:
            print(
                f"Processed {checked} row(s): {touched} updated, {skipped} skipped, "
                f"{len(failures)} failed.",
                file=sys.stderr,
            )
        return 1

    if not args.errors_only:
        print(f"Processed {checked} row(s): {touched} updated, {skipped} skipped.")

    return 0


def main() -> int:
    args = _parse_args()
    technical_drawing_mode = DraftingMode(args.technical_drawing_mode)
    os.environ[TECHNICAL_DRAWING_MODE_ENV] = technical_drawing_mode.value
    return asyncio.run(_async_main(args))


if __name__ == "__main__":
    raise SystemExit(main())
