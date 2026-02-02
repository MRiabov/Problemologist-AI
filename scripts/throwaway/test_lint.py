from build123d import *
import unused_module


def build(seed, scale=(1.0, 1.0, 1.0)):
    with BuildPart() as p:
        Box(10, 10, 10)
        # Undefined variable
        missing_var = undefined_name

    return "MOCK_MJCF"


# Syntax error
if x == 5:
    pass
