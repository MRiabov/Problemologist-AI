from __future__ import annotations

from build123d import Compound

from utils.metadata import CompoundMetadata


# Add authored benchmark fixtures to this children list.
# Every top-level child you add here must have a unique label, and that label
# must not be `environment`, start with `zone_`, or start with
# `benchmark_payload__` because the simulator reserves those names for the
# scene root and generated objective bodies.
def build() -> Compound:
    """Return the benchmark assembly geometry for this workspace."""
    # NOTE: Do NOT include the payload here — the simulation system spawns
    # `benchmark_payload__projectile_ball` independently from
    # `benchmark_definition.yaml`. Returning it from build() creates a duplicate
    # body that collides with the spawned ball, causing instant OUT_OF_BOUNDS.
    environment = Compound(children=[])
    environment.label = "benchmark_environment"
    environment.metadata = CompoundMetadata()
    return environment


result = build()
