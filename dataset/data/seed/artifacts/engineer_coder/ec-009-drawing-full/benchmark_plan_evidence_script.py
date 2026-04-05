from build123d import Align, Box, Compound

from utils.metadata import CompoundMetadata, PartMetadata


def build():
    env = Box(150.0, 150.0, 10.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    env.label = "environment_fixture"
    env.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    subassembly = Compound(label="benchmark_environment", children=[env])
    subassembly.metadata = CompoundMetadata()
    assembly = Compound(children=[subassembly])
    assembly.metadata = CompoundMetadata()
    return assembly
