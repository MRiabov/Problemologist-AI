from build123d import Align, Box

from utils.metadata import PartMetadata


def build():
    part = Box(10.0, 10.0, 10.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part.label = "environment_fixture"
    part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return part
