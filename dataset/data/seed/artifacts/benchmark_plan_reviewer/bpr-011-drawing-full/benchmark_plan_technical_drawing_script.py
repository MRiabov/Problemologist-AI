from build123d import Box

# The `raised_goal_shelf` geometry is designed to occupy the goal zone so the object must rest there to score.


def build():
    part = Box(6, 6, 2)
    part.label = "benchmark_plan_technical_drawing"
    return part
