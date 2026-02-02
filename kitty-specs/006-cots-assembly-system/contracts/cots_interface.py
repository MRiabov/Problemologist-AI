from dataclasses import dataclass
from typing import Any


@dataclass
class PartSummary:
    id: str
    name: str
    provider: str


@dataclass
class PartPreview:
    id: str
    image_path: str
    description: str
    metadata: dict[str, Any]
    recipe: str


class PartProvider:
    """Interface for a source of COTS parts."""

    def search(self, query: str) -> list[PartSummary]:
        """Return parts matching the query string."""
        pass

    def get_preview(self, part_id: str) -> PartPreview:
        """Return full details and assets for a specific part."""
        pass


class PartIndex:
    """Central registry for COTS parts."""

    def register_provider(self, provider: PartProvider):
        """Add a new source of parts."""
        pass

    def search(self, query: str) -> list[PartSummary]:
        """Aggregate search across all providers."""
        pass

    def preview(self, part_id: str) -> PartPreview:
        """Get preview from the owning provider."""
        pass
