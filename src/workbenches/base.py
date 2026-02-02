from abc import ABC, abstractmethod

from build123d import Part


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

    @abstractmethod
    def calculate_cost(self, part: Part) -> float:
        """
        Calculates the cost of producing the part according to this workbench's cost model.

        Args:
            part: The build123d Part to cost.

        Returns:
            The calculated cost as a float.
        """
        pass
