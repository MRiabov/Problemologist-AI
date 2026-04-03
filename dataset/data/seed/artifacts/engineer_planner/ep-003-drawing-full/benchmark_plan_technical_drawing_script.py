from build123d import Align, Box, Location, TechnicalDrawing

from utils.metadata import PartMetadata


def build():
    TechnicalDrawing(title="Seeded benchmark drafting")
    part = Box(10.0, 10.0, 10.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part.label = "environment_fixture"
    part = part.moved(Location((-220.0, -150.0, 0.0)))
    part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    return part
