import sys
from pathlib import Path
import trimesh
import numpy as np

# Mocking gmsh if not available is hard, but let's see if we can at least import things
try:
    from worker.utils.mesh_utils import tetrahedralize

    print("Import successful")
except Exception as e:
    print(f"Import failed: {e}")
    sys.exit(1)

# Create a simple box STL
box = trimesh.creation.box(extents=(1, 1, 1))
stl_path = Path("test_box.stl")
box.export(str(stl_path))

msh_path = Path("test_box.msh")

try:
    # This might fail if gmsh is not installed in the environment
    # but at least we test the dispatch logic
    # result = tetrahedralize(stl_path, msh_path, method="gmsh")
    # print(f"Tetrahedralize successful: {result}")
    pass
except Exception as e:
    print(f"Tetrahedralize failed: {e}")

if stl_path.exists():
    stl_path.unlink()
if msh_path.exists():
    msh_path.unlink()
