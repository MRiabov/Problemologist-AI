from build123d import Box, BuildPart

with BuildPart() as builder:
    Box(10, 10, 10)

result = builder.part
