import random
from build123d import (
    Box,
    BuildPart,
    Compound,
    Locations,
    Hole,
    CounterSinkHole,
    CounterBoreHole,
    Axis,
    fillet,
)


def build(seed: int = 0, scale: float = 1.0) -> Compound:
    """
    Builds the CAD component based on a seed and scale.
    """
    random.seed(seed)

    # Randomized dimensions for the base box
    length = random.uniform(50, 100) * scale
    width = random.uniform(50, 100) * scale
    height = random.uniform(5, 15) * scale

    with BuildPart() as p:
        Box(length, width, height)

        # Add a random number of holes on the top face
        num_holes = random.randint(2, 6)
        top_face = p.faces().sort_by(Axis.Z)[-1]

        with Locations(top_face):
            for _ in range(num_holes):
                # Random position on the top face
                x = random.uniform(-length * 0.4, length * 0.4)
                y = random.uniform(-width * 0.4, width * 0.4)
                radius = random.uniform(2, 5) * scale

                with Locations((x, y)):
                    # Randomly choose between a simple hole, counter-sink or counter-bore
                    hole_type = random.choice(["simple", "sink", "bore"])
                    if hole_type == "simple":
                        Hole(radius=radius)
                    elif hole_type == "sink":
                        CounterSinkHole(radius=radius, counter_sink_radius=radius * 1.5)
                    else:
                        CounterBoreHole(
                            radius=radius,
                            counter_bore_radius=radius * 1.5,
                            counter_bore_depth=height * 0.3,
                        )

        # Apply randomized fillets to the top edges
        if random.random() > 0.3:
            fillet_radius = random.uniform(0.5, 2.0) * scale
            fillet(p.faces().sort_by(Axis.Z)[-1].edges(), radius=fillet_radius)

    return p.part


if __name__ == "__main__":
    for s in [42, 123]:
        component = build(seed=s, scale=1.0)
        print(f"Seed {s}, Volume: {component.volume}")
