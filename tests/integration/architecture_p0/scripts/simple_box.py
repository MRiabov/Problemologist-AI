from typing import Any

from build123d import Align, Box, BuildPart, Compound
from shared.models.schemas import PartMetadata


def build(_params: dict[str, Any] | None = None) -> Compound:
    # A simple box that intersects with a forbidden zone (if placed there)
    # or a goal zone.
    # The actual placement depends on objectives.yaml, but this script
    # just returns a simple shape.
    with BuildPart() as p:
        Box(1, 1, 1, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p.part.label = "target_box"
    p.part.metadata = PartMetadata(material_id="aluminum_6061", fixed=False)
    return p.part
