import random
from build123d import Box, BuildPart, Compound


def build(seed: int = 0, scale: float = 1.0) -> Compound:
    """
    Builds the CAD component based on a seed and scale.
    """
    # Use seed for randomization
    rng = random.Random(seed)

    with BuildPart() as p:
        x_dim = rng.uniform(5.0, 15.0) * scale
        y_dim = rng.uniform(5.0, 15.0) * scale
        z_dim = rng.uniform(5.0, 15.0) * scale
        Box(x_dim, y_dim, z_dim)
    return p.part


if __name__ == "__main__":
    component1 = build(seed=42, scale=1.0)
    print(f"Volume with seed 42: {component1.volume}")
    component2 = build(seed=43, scale=1.0)
    print(f"Volume with seed 43: {component2.volume}")
