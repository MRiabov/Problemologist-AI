from build123d import Box, BuildPart, Compound
import random

def build(seed: int = 0, scale: float = 1.0) -> Compound:
    """
    Builds the CAD component based on a seed and scale.
    """
    # Use seed for randomization
    rng = random.Random(seed)

    # Randomized dimensions around base 10
    length = 10 * scale * rng.uniform(0.8, 1.2)
    width = 10 * scale * rng.uniform(0.8, 1.2)
    height = 10 * scale * rng.uniform(0.8, 1.2)

    with BuildPart() as p:
        Box(length, width, height)
    return p.part


if __name__ == "__main__":
    component = build(seed=42, scale=1.0)
    print(f"Volume: {component.volume}")
