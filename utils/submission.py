from build123d import Compound

from shared.utils.agent import (
    simulate_benchmark as _simulate_benchmark,
)
from shared.utils.agent import (
    simulate_engineering as _simulate_engineering,
)
from shared.utils.agent import (
    submit_benchmark_for_review as _submit_benchmark_for_review,
)
from shared.utils.agent import (
    submit_solution_for_review as _submit_solution_for_review,
)
from shared.utils.agent import (
    validate_benchmark as _validate_benchmark,
)
from shared.utils.agent import (
    validate_engineering as _validate_engineering,
)
from shared.workers.schema import BenchmarkToolResponse


def validate_benchmark(compound: Compound, **kwargs) -> tuple[bool, str | None]:
    return _validate_benchmark(compound, **kwargs)


def validate_engineering(compound: Compound, **kwargs) -> tuple[bool, str | None]:
    return _validate_engineering(compound, **kwargs)


def simulate_benchmark(compound: Compound, **kwargs) -> BenchmarkToolResponse:
    return _simulate_benchmark(compound, **kwargs)


def simulate_engineering(compound: Compound, **kwargs) -> BenchmarkToolResponse:
    return _simulate_engineering(compound, **kwargs)


def submit_benchmark_for_review(compound: Compound) -> bool:
    return _submit_benchmark_for_review(compound)


def submit_solution_for_review(compound: Compound) -> bool:
    return _submit_solution_for_review(compound)


__all__ = [
    "simulate_benchmark",
    "simulate_engineering",
    "submit_benchmark_for_review",
    "submit_solution_for_review",
    "validate_benchmark",
    "validate_engineering",
]
