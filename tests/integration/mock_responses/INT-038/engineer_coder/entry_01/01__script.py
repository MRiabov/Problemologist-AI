# ruff: noqa
from build123d import *  # noqa: F403, F405

from shared.models.schemas import PartMetadata
from shared.workers.workbench_models import ManufacturingMethod


def build():
    p = Box(0.2, 0.2, 0.2)
    p = p.moved(Location((6, 6, 6)))
    p.label = "obj"
    p.metadata = PartMetadata(
        material_id="abs",
        fixed=True,
        manufacturing_method=ManufacturingMethod.THREE_DP,
    )
    return p
