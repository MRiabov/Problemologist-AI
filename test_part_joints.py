from build123d import Part, Box, RigidJoint, Location

p = Box(10, 10, 10)
print(f"Has joints? {hasattr(p, 'joints')}")
print(f"Part dir: {dir(p)}")

# Try standard pattern
try:
    with BuildPart() as bp:
        Box(10, 10, 10)
        RigidJoint("test", Location((0, 0, 0)))

    print(f"BP part joints: {bp.part.joints}")
except NameError:
    print("BuildPart not imported")
except Exception as e:
    print(f"BuildPart error: {e}")
