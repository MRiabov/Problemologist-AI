from build123d import Box


def build():
    part = Box(6, 6, 2)
    part.label = "benchmark_plan_technical_drawing"
    return part
