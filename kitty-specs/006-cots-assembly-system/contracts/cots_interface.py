from typing import List, Dict, Optional, Any
from dataclasses import dataclass

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
    metadata: Dict[str, Any]
    recipe: str

class PartProvider:
    """Interface for a source of COTS parts."""
    
    def search(self, query: str) -> List[PartSummary]:
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
        
    def search(self, query: str) -> List[PartSummary]:
        """Aggregate search across all providers."""
        pass
        
    def preview(self, part_id: str) -> PartPreview:
        """Get preview from the owning provider."""
        pass
