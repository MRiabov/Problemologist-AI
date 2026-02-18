import genesis as gs
import time
import torch
import os
from build123d import Box, Sphere, export_stl
import trimesh
from pathlib import Path


def measure_compilation():
    print("--- Genesis Compilation Baseline ---")

    # 1. Init time
    start_init = time.time()
    has_gpu = torch.cuda.is_available()
    backend = gs.gpu if has_gpu else gs.cpu
    print(f"Initializing Genesis on {'GPU' if has_gpu else 'CPU'}...")
    gs.init(backend=backend, logging_level="info")
    init_duration = time.time() - start_init
    print(f"gs.init duration: {init_duration:.2f}s")

    # 2. Primitive: Box
    print("\nPhase 1: Box Primitive")
    start_scene = time.time()
    scene = gs.Scene(show_viewer=False)
    scene.add_entity(gs.morphs.Plane())
    scene.add_entity(gs.morphs.Box(pos=(0, 0, 1), size=(1, 1, 1)))
    create_duration = time.time() - start_scene

    start_build = time.time()
    scene.build()
    build_duration = time.time() - start_build

    start_step = time.time()
    for _ in range(10):
        scene.step()
    step_duration = time.time() - start_step

    print(f"  Scene Creation: {create_duration:.2f}s")
    print(f"  Scene Build (Compilation): {build_duration:.2f}s")
    print(f"  First 10 steps: {step_duration:.2f}s")

    # 3. Primitive: Sphere (New primitive type)
    print("\nPhase 2: Sphere Primitive (Different type)")
    start_scene = time.time()
    scene = gs.Scene(show_viewer=False)
    scene.add_entity(gs.morphs.Plane())
    scene.add_entity(gs.morphs.Sphere(pos=(0, 0, 1), radius=1))
    create_duration = time.time() - start_scene

    start_build = time.time()
    scene.build()
    build_duration = time.time() - start_build

    print(f"  Scene Build (New Compilation?): {build_duration:.2f}s")

    # 4. Mesh: build123d export
    print("\nPhase 3: Mesh (build123d export)")
    # Create a temp mesh
    part = Box(10, 10, 10)
    stl_path = "temp_baseline.stl"
    obj_path = "temp_baseline.obj"
    export_stl(part, stl_path)
    mesh = trimesh.load(stl_path)
    mesh.export(obj_path)

    start_scene = time.time()
    scene = gs.Scene(show_viewer=False)
    scene.add_entity(gs.morphs.Plane())
    scene.add_entity(gs.morphs.Mesh(file=obj_path, pos=(0, 0, 1)))
    create_duration = time.time() - start_scene

    start_build = time.time()
    scene.build()
    build_duration = time.time() - start_build

    print(f"  Scene Build (Mesh Voxelization): {build_duration:.2f}s")

    # 5. Mesh: Sphere (Potential bottleneck)
    print("\nPhase 4: Sphere Mesh (build123d export)")
    part = Sphere(10)
    stl_path = "temp_sphere.stl"
    obj_path = "temp_sphere.obj"
    export_stl(part, stl_path)
    mesh = trimesh.load(stl_path)
    mesh.export(obj_path)

    start_scene = time.time()
    scene = gs.Scene(show_viewer=False)
    scene.add_entity(gs.morphs.Plane())
    scene.add_entity(gs.morphs.Mesh(file=obj_path, pos=(0, 0, 1)))
    create_duration = time.time() - start_scene

    start_build = time.time()
    scene.build()
    build_duration = time.time() - start_build

    print(f"  Scene Build (Sphere Mesh Voxelization): {build_duration:.2f}s")

    # Cleanup
    if os.path.exists(stl_path):
        os.remove(stl_path)
    if os.path.exists(obj_path):
        os.remove(obj_path)


if __name__ == "__main__":
    measure_compilation()
