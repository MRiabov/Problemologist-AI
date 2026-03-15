from build123d import Box

from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod


def build():
    p = Box(1, 1, 1)
    p.label = "batch_cube"
    p.metadata = PartMetadata(
        material_id="abs",
        fixed=True,
        manufacturing_method=ManufacturingMethod.THREE_DP,
    )
    return p
