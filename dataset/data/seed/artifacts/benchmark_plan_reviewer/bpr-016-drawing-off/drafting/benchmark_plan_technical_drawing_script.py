from build123d import Box

from utils.metadata import PartMetadata


def build():
    part = Box(6, 6, 2)
    part.label = "benchmark_plan_technical_drawing"
    part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return part
