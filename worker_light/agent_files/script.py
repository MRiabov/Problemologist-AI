import random
from build123d import Box, BuildPart, Compound


def build(seed: int = 0, scale: float = 1.0) -> Compound:
    """
    Builds the CAD component based on a seed and scale.
    """
    # Use seed for randomization
    random.seed(seed)

    # Randomize dimensions
    length = random.uniform(5, 15) * scale
    width = random.uniform(5, 15) * scale
    height = random.uniform(5, 15) * scale

    with BuildPart() as p:
        Box(length, width, height)
    return p.part


if __name__ == "__main__":
    component = build(seed=42, scale=1.0)
    print(f"Volume: {component.volume}")
