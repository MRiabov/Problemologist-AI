from build123d import Part, Box

p = Box(10, 10, 10)
print(f"'joints' in dir(p): {'joints' in dir(p)}")
print(f"type(p.joints) if exists: {type(getattr(p, 'joints', None))}")
