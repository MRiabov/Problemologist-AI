from abc import ABC, abstractmethod
from typing import List, Union, Dict, Any

from build123d import Part


class Workbench(ABC):
    """
    Abstract Base Class for all Workbenches in the Agentic CAD Environment.
    A Workbench defines a set of constraints and a cost model for a specific
    manufacturing or assembly process.
    """

    @abstractmethod
    def validate(self, part: Part) -> List[Union[Exception, str]]:
        """
        Validates the part against the workbench constraints.

        Args:
            part: The build123d Part to validate.

        Returns:
            A list of violations (strings or exceptions). If empty, the part is valid.
        """
        pass

    def validate_geometry(self, part: Part) -> List[Union[Exception, str]]:
        """
        Alias for validate, as per spec requirements.
        """
        return self.validate(part)

    @abstractmethod
    def calculate_cost(
        self, part: Part, quantity: int = 1, context: dict | None = None
    ) -> Union[float, Dict[str, Any]]:
        """
        Calculates the cost of producing the part according to this workbench's cost model.

        Args:
            part: The build123d Part to cost.
            quantity: The number of parts to produce.
            context: Optional dictionary for tracking reused parts across an assembly.
                     Keyed by part hash, value could be number of times seen.

        Returns:
            The calculated cost as a float OR a dictionary with detailed breakdown.
            If a dictionary, it MUST contain a "total_cost" key.
        """
        pass
