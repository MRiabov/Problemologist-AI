from typing import Any

from build123d import Box, BuildPart, Compound


def build(_params: dict[str, Any] | None = None) -> Compound:
    # A simple box that intersects with a forbidden zone (if placed there)
    # or a goal zone.
    # The actual placement depends on objectives.yaml, but this script
    # just returns a simple shape.
    with BuildPart() as p:
        Box(1, 1, 1)
    return p.part
