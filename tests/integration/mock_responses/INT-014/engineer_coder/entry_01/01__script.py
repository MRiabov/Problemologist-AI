from build123d import Box, Pos

from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod


def build():
    part = Box(1, 1, 1).move(Pos(0, 0, 0.5))
    part.label = "motor_mount"
    part.metadata = PartMetadata(
        material_id="abs",
        fixed=True,
        manufacturing_method=ManufacturingMethod.THREE_DP,
    )
    return part
