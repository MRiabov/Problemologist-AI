from pathlib import Path

import dspy
import pytest

from controller.agent.benchmark.data_loader import load_benchmark_dataset


def test_load_benchmark_coder():
    # Ensure the file exists before testing, or skip if environment is not set up
    if not (
        Path("evals/datasets/benchmark_coder.json").exists()
        or Path(__file__)
        .parent.parent.parent.parent.joinpath("evals/datasets/benchmark_coder.json")
        .exists()
    ):
        pytest.skip("benchmark_coder.json not found")

    dataset = load_benchmark_dataset("benchmark_coder")
    assert len(dataset) > 0
    assert isinstance(dataset[0], dspy.Example)
    assert hasattr(dataset[0], "task")
    assert dataset[0].agent_name == "benchmark_coder"
    assert "task" in dataset[0]._input_keys


def test_load_nonexistent_dataset():
    with pytest.raises(FileNotFoundError):
        load_benchmark_dataset("nonexistent_dataset_12345")
