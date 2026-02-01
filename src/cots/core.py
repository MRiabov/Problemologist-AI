from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

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
    metadata: Dict[str, Any] = field(default_factory=dict)
    recipe: str = ""

@dataclass
class Part:
    id: str
    factory: Callable
    params: Dict[str, Any] = field(default_factory=dict)

class PartIndex:
    def __init__(self):
        self.providers: Dict[str, Any] = {}

    def register_provider(self, name: str, provider: Any):
        """Register a new part provider."""
        self.providers[name] = provider

    def search(self, query: str) -> List[PartSummary]:
        """Aggregate search results from all registered providers."""
        results = []
        for provider in self.providers.values():
            results.extend(provider.search(query))
        return results

    def _get_provider_for_id(self, part_id: str):
        """Extract provider name from ID and return the provider instance."""
        if ":" not in part_id:
            raise ValueError(f"Invalid part ID: {part_id}. Expected 'provider:id' format.")
        
        provider_name, _ = part_id.split(":", 1)
        if provider_name not in self.providers:
            raise ValueError(f"No provider registered with name: {provider_name}")
        
        return self.providers[provider_name]

    def get_part(self, part_id: str) -> Part:
        """Retrieve a Part by its namespaced ID."""
        provider = self._get_provider_for_id(part_id)
        return provider.get_part(part_id)

    def preview(self, part_id: str) -> PartPreview:
        """Retrieve a PartPreview by its namespaced ID."""
        provider = self._get_provider_for_id(part_id)
        return provider.get_preview(part_id)
