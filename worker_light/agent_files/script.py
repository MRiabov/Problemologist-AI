import random

from build123d import Box, BuildPart, Compound


def build(seed: int = 0, scale: float = 1.0) -> Compound:
    """
    Builds the CAD component based on a seed and scale.
    """
    # Use seed for randomization
    random.seed(seed)

    # Randomized geometry
    x_dim = random.uniform(8.0, 12.0) * scale
    y_dim = random.uniform(8.0, 12.0) * scale
    z_dim = random.uniform(8.0, 12.0) * scale

    with BuildPart() as p:
        Box(x_dim, y_dim, z_dim)
    return p.part


if __name__ == "__main__":
    component = build(seed=42, scale=1.0)
    print(f"Volume: {component.volume}")
