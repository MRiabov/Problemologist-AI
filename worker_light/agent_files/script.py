from build123d import Box, BuildPart, Compound


def build(seed: int = 0, scale: float = 1.0) -> Compound:
    """
    Builds the CAD component based on a seed and scale.
    """
    import random

    # Use seed for randomization
    random.seed(seed)

    # Implement randomized geometry
    x_dim = 10 * scale * random.uniform(0.8, 1.2)
    y_dim = 10 * scale * random.uniform(0.8, 1.2)
    z_dim = 10 * scale * random.uniform(0.8, 1.2)

    with BuildPart() as p:
        Box(x_dim, y_dim, z_dim)
    return p.part


if __name__ == "__main__":
    component = build(seed=42, scale=1.0)
    print(f"Volume: {component.volume}")
