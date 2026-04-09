from build123d import Box

# The `lower_bin` geometry is designed to occupy the goal zone so the object must rest there to score.


def build():
    part = Box(6, 6, 2)
    part.label = "benchmark_plan_evidence"
    return part
