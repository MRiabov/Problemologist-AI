from build123d import Box

from shared.enums import ManufacturingMethod
from shared.models.schemas import PartMetadata


def build():
    part = Box(1, 1, 1).translate((7.5, 7.5, 7.5))
    part.label = "projectile_ball"
    part.metadata = PartMetadata(
        material_id="abs",
        manufacturing_method=ManufacturingMethod.THREE_DP,
    )
    return part
