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

    from .models import BenchmarkItem

    examples = []
    for item_raw in data:
        item = BenchmarkItem(**item_raw)
        inputs = {}
        # Convert model back to dict for dspy.Example, but validated
        item_dict = item.model_dump()
        for k, v in item_dict.items():
            if k in ["task", "prompt", "context", "expected_criteria"]:
                inputs[k] = v
            elif k in ["objectives", "goals", "constraints"]:
                inputs["objectives"] = v

        if "objectives" not in inputs:
            inputs["objectives"] = {"max_unit_cost": 100.0, "max_weight": 50.0}

        # Create the example
        example = dspy.Example(**inputs).with_inputs(*inputs.keys())
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
