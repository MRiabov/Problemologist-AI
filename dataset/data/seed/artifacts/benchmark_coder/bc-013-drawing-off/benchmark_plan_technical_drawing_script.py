"""Benchmark plan technical drawing script for the side-deflect ramp benchmark."""
from build123d import Box, TechnicalDrawing

from utils.metadata import PartMetadata


def build() -> TechnicalDrawing:
    """Return a TechnicalDrawing package for the approved planner inventory."""
    model = Box(6, 6, 2)
    model.label = "benchmark_plan_technical_drawing"
    model.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    drawing = TechnicalDrawing(
        model,
        title="Side-Deflect Ramp Benchmark Plan",
    )
    return drawing
