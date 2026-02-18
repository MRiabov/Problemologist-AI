import json
from pathlib import Path

import dspy
import structlog

logger = structlog.get_logger(__name__)


def load_benchmark_dataset(agent_type: str) -> list[dspy.Example]:
    """
    Loads a benchmark dataset from evals/datasets/ and converts to dspy.Example objects.
    """
    dataset_path = Path("evals/datasets") / f"{agent_type}.json"

    if not dataset_path.exists():
        root_path = Path(__file__).parent.parent.parent.parent
        dataset_path = root_path / "evals" / "datasets" / f"{agent_type}.json"

    if not dataset_path.exists():
        raise FileNotFoundError(f"Dataset not found at {dataset_path}")

    with dataset_path.open("r") as f:
        data = json.load(f)

    examples = []
    for item in data:
        inputs = {}
        for k, v in item.items():
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


def load_langfuse_traces(agent_name: str, limit: int = 20) -> list[dspy.Example]:
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
        logger.error("langfuse_fetch_failed", agent_name=agent_name, error=str(e))
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
