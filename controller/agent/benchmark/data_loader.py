import json
from pathlib import Path

import dspy
import structlog

from shared.enums import AgentName

logger = structlog.get_logger(__name__)


def load_benchmark_dataset(agent_type: AgentName) -> list[dspy.Example]:
    """
    Loads a benchmark dataset and converts it to dspy.Example objects.
    """
    root_path = Path(__file__).parent.parent.parent.parent
    candidate_paths = (
        Path("dataset/data/seed/role_based") / f"{agent_type}.json",
        root_path / "dataset" / "data" / "seed" / "role_based" / f"{agent_type}.json",
        Path("evals/datasets") / f"{agent_type}.json",
        root_path / "evals" / "datasets" / f"{agent_type}.json",
    )
    dataset_path = next((path for path in candidate_paths if path.exists()), None)

    if dataset_path is None:
        searched_paths = ", ".join(str(path) for path in candidate_paths)
        raise FileNotFoundError(
            f"Dataset not found in any expected location: {searched_paths}"
        )

    with dataset_path.open("r") as f:
        data = json.load(f)

    from .models import BenchmarkExampleInputs, BenchmarkItem

    examples = []
    for item_raw in data:
        item = BenchmarkItem(**item_raw)
        # Preserve the validated schema fields as a typed transfer object.
        inputs = BenchmarkExampleInputs(
            task=item.task,
            prompt=item.prompt,
            context=item.context,
            expected_criteria=item.expected_criteria,
            objectives=item.objectives or {"max_unit_cost": 100.0, "max_weight": 50.0},
            goals=item.goals,
            constraints=item.constraints,
        )
        payload = inputs.model_dump()

        # Create the example
        example = dspy.Example(**payload).with_inputs(*payload.keys())
        example.agent_name = agent_type
        examples.append(example)

    return examples


def load_langfuse_traces(agent_name: AgentName, limit: int = 20) -> list[dspy.Example]:
    """
    Fetches historical traces from Langfuse and converts them to dspy.Example objects.
    """
    from controller.observability.langfuse import get_langfuse_client

    client = get_langfuse_client()
    if not client:
        logger.warning("langfuse_client_not_available")
        return []

    try:
        traces = client.get_traces(name=agent_name, limit=limit).data
    except Exception as e:
        logger.warning("langfuse_fetch_failed", agent_name=agent_name, error=str(e))
        return []

    examples = []
    for trace in traces:
        in_data = (
            trace.input
            if isinstance(trace.input, dict)
            else {"input": str(trace.input)}
        )

        if "objectives" not in in_data:
            metadata = trace.metadata or {}
            if "objectives" in metadata:
                in_data["objectives"] = metadata["objectives"]

        example = dspy.Example(**in_data).with_inputs(*in_data.keys())
        example.agent_name = agent_name
        example.langfuse_trace_id = trace.id
        examples.append(example)

    logger.info("langfuse_traces_loaded", count=len(examples), agent_name=agent_name)
    return examples
