import random
from build123d import Box, BuildPart, Compound


def build(seed: int = 0, scale: float = 1.0) -> Compound:
    """
    Builds the CAD component based on a seed and scale.
    """
    # Use seed for randomization
    rng = random.Random(seed)

    # Randomized dimensions
    width = rng.uniform(5.0, 15.0) * scale
    length = rng.uniform(5.0, 15.0) * scale
    height = rng.uniform(5.0, 15.0) * scale

    with BuildPart() as p:
        Box(width, length, height)
    return p.part


if __name__ == "__main__":
    component = build(seed=42, scale=1.0)
    print(f"Volume: {component.volume}")
