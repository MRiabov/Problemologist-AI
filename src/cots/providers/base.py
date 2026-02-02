from abc import ABC, abstractmethod
from typing import List
from src.cots.core import Part, PartPreview, PartSummary


class PartProvider(ABC):
    @abstractmethod
    def search(self, query: str) -> List[PartSummary]:
        """Search for parts matching the query."""
        pass

    @abstractmethod
    def get_part(self, part_id: str) -> Part:
        """Retrieve a Part object by its ID."""
        pass

    @abstractmethod
    def get_preview(self, part_id: str) -> PartPreview:
        """Retrieve a PartPreview object by its ID."""
        pass
