from build123d import Box, BuildPart

# COTS import hint:
# from shared.cots.parts.motors import ServoMotor
# motor = ServoMotor.from_catalog_id("ServoMotor_DS3218")


def build():
    with BuildPart() as builder:
        Box(10, 10, 10)
    return builder.part


result = build()
