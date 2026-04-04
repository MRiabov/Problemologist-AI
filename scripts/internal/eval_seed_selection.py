from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from evals.logic.dataset_selection import (
    filter_rows_by_technical_drawing_mode,  # noqa: E402
)
from evals.logic.models import EvalDatasetItem  # noqa: E402
from shared.agents.config import DraftingMode  # noqa: E402
from shared.enums import AgentName  # noqa: E402


def load_seed_dataset(
    agent: AgentName,
    *,
    task_id: str | None,
    limit: int,
    levels: set[int] | None,
    technical_drawing_mode: DraftingMode,
    root: Path = ROOT,
) -> list[EvalDatasetItem]:
    dataset_roots = [
        root / "dataset" / "evals" / "datasets",
        root / "dataset" / "data" / "seed" / "role_based",
    ]
    json_path = next(
        (
            dataset_root / f"{agent.value}.json"
            for dataset_root in dataset_roots
            if (dataset_root / f"{agent.value}.json").exists()
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
    data = filter_rows_by_technical_drawing_mode(
        data, technical_drawing_mode=technical_drawing_mode
    )
    if limit > 0:
        data = data[:limit]

    seed_dataset = json_path.relative_to(root)
    return [
        EvalDatasetItem.model_validate({**item_raw, "seed_dataset": seed_dataset})
        for item_raw in data
    ]
