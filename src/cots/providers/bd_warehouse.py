from typing import List, Dict, Any
from src.cots.providers.base import PartProvider
from src.cots.core import Part, PartSummary, PartPreview
from bd_warehouse.open_builds import StepperMotor
from bd_warehouse.bearing import SingleRowDeepGrooveBallBearing
from bd_warehouse.fastener import SocketHeadCapScrew, HexNut

class BDWarehouseProvider(PartProvider):
    def __init__(self):
        self.parts: Dict[str, Part] = {}
        self.summaries: List[PartSummary] = []
        self._index_motors()
        self._index_bearings()
        self._index_fasteners()

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

    def _index_bearings(self):
        # Using SingleRowDeepGrooveBallBearing as standard
        bearing_type = "SKT"
        try:
            sizes = SingleRowDeepGrooveBallBearing.sizes(bearing_type)
            # Limit to common sizes to avoid indexing bloat
            common_sizes = [s for s in sizes if any(ms in s for ms in ["M3", "M4", "M5", "M8"])]
            
            for size in common_sizes:
                part_id = f"bd_warehouse:bearing:{size}"
                self.summaries.append(PartSummary(id=part_id, name=f"Bearing {size}", provider="bd_warehouse"))
                
                self.parts[part_id] = Part(
                    id=part_id,
                    factory=lambda s=size: SingleRowDeepGrooveBallBearing(size=s, bearing_type=bearing_type),
                    params={"size": size, "type": "bearing", "bearing_type": bearing_type}
                )
        except Exception as e:
            print(f"Warning: Failed to index bearings: {e}")

    def _index_fasteners(self):
        # Index Screws (SocketHeadCapScrew)
        try:
            screw_type = "iso4762"
            sizes = SocketHeadCapScrew.sizes(screw_type)
            # M3, M4, M5 are most common
            common_sizes = [s for s in sizes if s.startswith(("M3-", "M4-", "M5-"))]
            
            for size in common_sizes:
                # Add a default length for indexing
                default_length = 10
                part_id = f"bd_warehouse:fastener:screw:{size}-{default_length}"
                self.summaries.append(PartSummary(id=part_id, name=f"Screw {size}x{default_length}", provider="bd_warehouse"))
                
                self.parts[part_id] = Part(
                    id=part_id,
                    factory=lambda s=size, l=default_length: SocketHeadCapScrew(size=s, length=l, fastener_type=screw_type),
                    params={"size": size, "length": default_length, "type": "fastener", "subtype": "screw", "fastener_type": screw_type}
                )
        except Exception as e:
            print(f"Warning: Failed to index screws: {e}")

        # Index Nuts (HexNut)
        try:
            nut_type = "iso4032"
            sizes = HexNut.sizes(nut_type)
            common_sizes = [s for s in sizes if s.startswith(("M3-", "M4-", "M5-"))]
            
            for size in common_sizes:
                part_id = f"bd_warehouse:fastener:nut:{size}"
                self.summaries.append(PartSummary(id=part_id, name=f"Nut {size}", provider="bd_warehouse"))
                
                self.parts[part_id] = Part(
                    id=part_id,
                    factory=lambda s=size: HexNut(size=s, fastener_type=nut_type),
                    params={"size": size, "type": "fastener", "subtype": "nut", "fastener_type": nut_type}
                )
        except Exception as e:
            print(f"Warning: Failed to index nuts: {e}")

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
        params = part.params
        
        # Recipe generation
        if params.get("type") == "motor":
            recipe = f'from bd_warehouse.open_builds import StepperMotor\npart = StepperMotor("{params["size"]}")'
        elif params.get("type") == "bearing":
            recipe = f'from bd_warehouse.bearing import SingleRowDeepGrooveBallBearing\npart = SingleRowDeepGrooveBallBearing(size="{params["size"]}", bearing_type="{params["bearing_type"]}")'
        elif params.get("type") == "fastener":
            if params.get("subtype") == "screw":
                recipe = f'from bd_warehouse.fastener import SocketHeadCapScrew\npart = SocketHeadCapScrew(size="{params["size"]}", length={params["length"]}, fastener_type="{params["fastener_type"]}")'
            elif params.get("subtype") == "nut":
                recipe = f'from bd_warehouse.fastener import HexNut\npart = HexNut(size="{params["size"]}", fastener_type="{params["fastener_type"]}")'
            else:
                recipe = "# Unknown fastener subtype"
        else:
            recipe = "# Recipe not implemented"

        return PartPreview(
            id=part_id,
            image_path="", # Placeholder for WP04
            description=f"Standard {params.get('size', 'Part')}",
            recipe=recipe
        )