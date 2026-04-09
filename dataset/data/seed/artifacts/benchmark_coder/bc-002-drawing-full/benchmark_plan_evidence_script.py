"""Benchmark plan evidence script for the tunnel-guide benchmark.

Builds the environment_fixture geometry within the benchmark build zone.
"""

from build123d import Align, Box, Compound

from utils.metadata import CompoundMetadata, PartMetadata


def build() -> Compound:
    """Build the environment fixture matching the approved planner inventory."""
    children = []

    # Tunnel body within build zone: X[-0.46, 0.45], Y[-0.12, 0.12], Z[0, 0.18]
    # Keep tunnel short enough to not overlap goal zone at X[0.42, 0.54]
    # Tunnel ends at X=0.40, well before goal zone at X=0.42
    tunnel = Box(0.8, 0.24, 0.15, align=(Align.CENTER, Align.CENTER, Align.MIN))
    tunnel.label = "environment_fixture"
    tunnel.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
    children.append(tunnel)

    asm = Compound(children=children)
    asm.label = "benchmark_plan_evidence"
    asm.metadata = CompoundMetadata()
    return asm
