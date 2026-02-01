from typing import List, Dict, Any
from src.cots.providers.base import PartProvider
from src.cots.core import Part, PartSummary, PartPreview
from bd_warehouse.open_builds import StepperMotor

class BDWarehouseProvider(PartProvider):
    def __init__(self):
        self.parts: Dict[str, Part] = {}
        self.summaries: List[PartSummary] = []
        self._index_motors()

    def _index_motors(self):
        # Hardcoded from bd_warehouse.open_builds.StepperMotor implementation
        sizes = ["Nema17", "Nema23", "Nema23HighTorque"]
        
        for size in sizes:
            part_id = f"bd_warehouse:motor:{size}"
            self.summaries.append(PartSummary(id=part_id, name=f"Stepper Motor {size}", provider="bd_warehouse"))
            
            self.parts[part_id] = Part(
                id=part_id,
                factory=lambda s=size: StepperMotor(s),
                params={"size": size, "type": "motor"}
            )

    def search(self, query: str) -> List[PartSummary]:
        return [s for s in self.summaries if query.lower() in s.name.lower() or query.lower() in s.id.lower()]

    def get_part(self, part_id: str) -> Part:
        if part_id not in self.parts:
            raise ValueError(f"Part not found: {part_id}")
        return self.parts[part_id]

    def get_preview(self, part_id: str) -> PartPreview:
        if part_id not in self.parts:
            raise ValueError(f"Part not found: {part_id}")
        
        part = self.parts[part_id]
        
        # Recipe generation
        if part.params.get("type") == "motor":
            size = part.params["size"]
            recipe = f'from bd_warehouse.open_builds import StepperMotor\npart = StepperMotor("{size}")'
        else:
            recipe = "# Recipe not implemented"

        return PartPreview(
            id=part_id,
            image_path="", # Placeholder for WP04
            description=f"Standard {part.params.get('size', 'Part')}",
            recipe=recipe
        )
