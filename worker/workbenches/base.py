from abc import ABC, abstractmethod
from collections.abc import Callable
from typing import Any, Protocol

from build123d import Compound, Part

from shared.type_checking import type_check
from worker.workbenches.models import (
    CostBreakdown,
    ManufacturingConfig,
    WorkbenchResult,
)

# Functional interface for workbench analysis
AnalyzeFunction = Callable[[Part | Compound, ManufacturingConfig, int], WorkbenchResult]


class WorkbenchAnalyzer(Protocol):
    """
    Protocol for functional workbench analysis.
    """

    def __call__(
        self, part: Part | Compound, config: ManufacturingConfig
    ) -> WorkbenchResult: ...


@type_check
class Workbench(ABC):
    """
    Abstract Base Class for all Workbenches in the Agentic CAD Environment.
    A Workbench defines a set of constraints and a cost model for a specific
    manufacturing or assembly process.
    """

    @abstractmethod
    def validate(self, part: Part) -> list[Exception | str]:
        """
        Validates the part against the workbench constraints.

        Args:
            part: The build123d Part to validate.

        Returns:
            A list of violations (strings or exceptions). If empty, the part is valid.
        """
        pass

    def validate_geometry(self, part: Part) -> list[Exception | str]:
        """
        Alias for validate, as per spec requirements.
        """
        return self.validate(part)

    @abstractmethod
    def calculate_cost(
        self,
        part: Part,
        quantity: int = 1,
        context: dict[str, Any] | None = None,
    ) -> CostBreakdown:
        """
        Calculates the cost of producing the part according to this workbench's cost model.

        Args:
            part: The build123d Part to cost.
            quantity: The number of parts to produce.
            context: Optional dictionary for tracking reused parts across an assembly.
                     Keyed by part hash, value could be number of times seen.

        Returns:
            The calculated cost as a CostBreakdown model.
        """
        pass
