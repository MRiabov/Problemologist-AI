"""Benchmark plan technical-drawing script for the tunnel-guide benchmark.

Produces an orthographic drawing package for the environment_fixture.
"""

from build123d import Align, Box, Compound, TechnicalDrawing

from utils.metadata import CompoundMetadata, PartMetadata


def build() -> Compound:
    """Return the drafted model for the approved planner inventory."""
    # Create the drawing artifact (side effect)
    TechnicalDrawing(title="Tunnel Guide Benchmark Plan")

    # Tunnel body within build zone: X[-0.46, 0.45], Y[-0.12, 0.12], Z[0, 0.18]
    # Keep tunnel width to 0.8m so it ends at X=0.40, before goal zone at X=0.42
    tunnel = Box(0.8, 0.24, 0.15, align=(Align.CENTER, Align.CENTER, Align.MIN))
    tunnel.label = "environment_fixture"
    tunnel.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)

    asm = Compound(children=[tunnel])
    asm.label = "benchmark_plan_technical_drawing"
    asm.metadata = CompoundMetadata()
    return asm
