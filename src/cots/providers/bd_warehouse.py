from bd_warehouse.bearing import SingleRowDeepGrooveBallBearing
from bd_warehouse.fastener import HexNut, SocketHeadCapScrew
from bd_warehouse.open_builds import CBeamLinearRail, StepperMotor, VSlotLinearRail

from src.cots.core import Part, PartPreview, PartSummary
from src.cots.providers.base import PartProvider
from src.cots.rendering import render_part
from src.cots.utils import get_description


class BDWarehouseProvider(PartProvider):
    def __init__(self):
        self.parts: dict[str, Part] = {}
        self.summaries: list[PartSummary] = []
        self._index_motors()
        self._index_bearings()
        self._index_fasteners()
        self._index_beams()

    def _index_motors(self):
        # Hardcoded from bd_warehouse.open_builds.StepperMotor implementation
        sizes = ["Nema17", "Nema23", "Nema23HighTorque"]

        for size in sizes:
            part_id = f"bd_warehouse:motor:{size}"
            self.summaries.append(
                PartSummary(
                    id=part_id, name=f"Stepper Motor {size}", provider="bd_warehouse"
                )
            )

            self.parts[part_id] = Part(
                id=part_id,
                factory=lambda s=size: StepperMotor(s),
                params={"size": size, "type": "motor"},
            )

    def _index_bearings(self):
        # Using SingleRowDeepGrooveBallBearing as standard
        bearing_type = "SKT"
        try:
            sizes = SingleRowDeepGrooveBallBearing.sizes(bearing_type)
            # Limit to common sizes to avoid indexing bloat
            common_sizes = [
                s for s in sizes if any(ms in s for ms in ["M3", "M4", "M5", "M8"])
            ]

            for size in common_sizes:
                part_id = f"bd_warehouse:bearing:{size}"
                self.summaries.append(
                    PartSummary(
                        id=part_id, name=f"Bearing {size}", provider="bd_warehouse"
                    )
                )

                self.parts[part_id] = Part(
                    id=part_id,
                    factory=lambda s=size: SingleRowDeepGrooveBallBearing(
                        size=s, bearing_type=bearing_type
                    ),
                    params={
                        "size": size,
                        "type": "bearing",
                        "bearing_type": bearing_type,
                    },
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
                self.summaries.append(
                    PartSummary(
                        id=part_id,
                        name=f"Screw {size}x{default_length}",
                        provider="bd_warehouse",
                    )
                )

                self.parts[part_id] = Part(
                    id=part_id,
                    factory=lambda s=size, l=default_length: SocketHeadCapScrew(
                        size=s, length=l, fastener_type=screw_type
                    ),
                    params={
                        "size": size,
                        "length": default_length,
                        "type": "fastener",
                        "subtype": "screw",
                        "fastener_type": screw_type,
                    },
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
                self.summaries.append(
                    PartSummary(id=part_id, name=f"Nut {size}", provider="bd_warehouse")
                )

                self.parts[part_id] = Part(
                    id=part_id,
                    factory=lambda s=size: HexNut(size=s, fastener_type=nut_type),
                    params={
                        "size": size,
                        "type": "fastener",
                        "subtype": "nut",
                        "fastener_type": nut_type,
                    },
                )
        except Exception as e:
            print(f"Warning: Failed to index nuts: {e}")

    def _index_beams(self):
        # Index V-Slot and C-Beam profiles
        try:
            # V-Slot Linear Rail
            v_slot_sizes = ["2020", "2040", "2060", "2080"]
            default_length = 250

            for size in v_slot_sizes:
                part_id = f"bd_warehouse:beam:v_slot:{size}-{default_length}"
                self.summaries.append(
                    PartSummary(
                        id=part_id,
                        name=f"V-Slot {size}x{default_length}",
                        provider="bd_warehouse",
                    )
                )

                rail_size = f"{size[:2]}x{size[2:]}"
                self.parts[part_id] = Part(
                    id=part_id,
                    factory=lambda rs=rail_size, l=default_length: VSlotLinearRail(
                        rail_size=rs, length=l
                    ),
                    params={
                        "size": size,
                        "rail_size": rail_size,
                        "length": default_length,
                        "type": "beam",
                        "subtype": "v_slot",
                    },
                )

            # C-Beam Linear Rail
            part_id = f"bd_warehouse:beam:c_beam:4080-{default_length}"
            self.summaries.append(
                PartSummary(
                    id=part_id,
                    name=f"C-Beam 4080x{default_length}",
                    provider="bd_warehouse",
                )
            )

            self.parts[part_id] = Part(
                id=part_id,
                factory=lambda l=default_length: CBeamLinearRail(length=l),
                params={
                    "size": "4080",
                    "length": default_length,
                    "type": "beam",
                    "subtype": "c_beam",
                },
            )
        except Exception as e:
            print(f"Warning: Failed to index beams: {e}")

    def search(self, query: str) -> list[PartSummary]:
        return [
            s
            for s in self.summaries
            if query.lower() in s.name.lower() or query.lower() in s.id.lower()
        ]

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
        elif params.get("type") == "beam":
            if params.get("subtype") == "v_slot":
                recipe = f'from bd_warehouse.open_builds import VSlotLinearRail\npart = VSlotLinearRail(rail_size="{params["rail_size"]}", length={params["length"]})'
            elif params.get("subtype") == "c_beam":
                recipe = f"from bd_warehouse.open_builds import CBeamLinearRail\npart = CBeamLinearRail(length={params['length']})"
            else:
                recipe = "# Unknown beam subtype"
        else:
            recipe = "# Recipe not implemented"

        # T015/T016: Load description and metadata
        desc_data = get_description(part_id)

        # T017: Render the part
        try:
            part_obj = part.factory()
            image_path = render_part(part_obj, part_id)
        except Exception as e:
            print(f"Warning: Failed to generate render for {part_id}: {e}")
            image_path = ""

        return PartPreview(
            id=part_id,
            image_path=image_path,
            description=desc_data["description"],
            metadata=desc_data.get("metadata", {}),
            recipe=recipe,
        )
