from build123d import Box, Location

from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod


def build():
    p = Box(0.2, 0.2, 0.2).move(Location((7.5, 7.5, 7.5)))
    p.label = "projectile_ball"
    p.metadata = PartMetadata(
        material_id="abs",
        fixed=False,
        manufacturing_method=ManufacturingMethod.THREE_DP,
    )
    return p
