from build123d import Compound

from shared.utils.agent import (
    simulate as _simulate,
)
from shared.utils.agent import (
    submit_for_review as _submit_for_review,
)
from shared.utils.agent import (
    validate as _validate,
)
from shared.workers.schema import BenchmarkToolResponse


def validate(compound: Compound, **kwargs) -> tuple[bool, str | None]:
    return _validate(compound, **kwargs)


def simulate(compound: Compound, **kwargs) -> BenchmarkToolResponse:
    return _simulate(compound, **kwargs)


def submit_for_review(compound: Compound) -> bool:
    return _submit_for_review(compound)


__all__ = ["simulate", "submit_for_review", "validate"]
