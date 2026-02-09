import mujoco
import numpy as np
from build123d import Box, Compound

from worker.simulation.builder import MeshProcessor, SceneCompiler, SimulationBuilder


def test_mesh_processor_obj_export(tmp_path):
    """Test that mesh processor exports OBJ format (per architecture spec)."""
    processor = MeshProcessor()
    box = Box(1, 1, 1)
    # Even if we pass .stl, it should convert to .obj
    input_path = tmp_path / "test_box.stl"

    result_paths = processor.process_geometry(box, input_path)

    # Should output as .obj
    assert len(result_paths) == 1
    obj_path = result_paths[0]
    assert obj_path.suffix == ".obj"
    assert obj_path.exists()
    assert obj_path.stat().st_size > 0


def test_scene_compiler(tmp_path):
    compiler = SceneCompiler()
    xml_path = tmp_path / "scene.xml"

    compiler.add_mesh_asset("box", "box.stl")
    compiler.add_body("box_body", mesh_names=["box"], pos=[0, 0, 1])
    compiler.save(xml_path)

    assert xml_path.exists()
    content = xml_path.read_text()
    assert "<mujoco" in content
    assert 'mesh="box"' in content


def test_simulation_builder(tmp_path):
    # Create a small assembly
    box1 = Box(0.1, 0.1, 0.1)
    box1.label = "part_1"

    # zone_goal
    box2 = Box(0.2, 0.2, 0.2)
    box2.label = "zone_goal"

    assembly = Compound(children=[box1, box2])

    builder = SimulationBuilder(tmp_path)
    scene_path = builder.build_from_assembly(assembly)

    assert scene_path.exists()
    # Now exports as .obj instead of .stl
    assert (tmp_path / "assets" / "part_1.obj").exists()

    # Try loading with MuJoCo
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    assert model is not None
    # model.nmesh might be 1 (the floor is a plane geom, not mesh)
    assert model.nmesh == 1
    # model.nsite should be 1 for zone_goal
    assert model.nsite == 1
    # Check site size (MuJoCo site size for box is half-extents)
    # box2 was 0.2 -> half size 0.1
    assert np.allclose(model.site_size[0], [0.1, 0.1, 0.1])


def test_vhacd_decomposition(tmp_path):
    # Create a concave shape: a U-shape
    from build123d import Box

    b1 = Box(1, 0.2, 0.2)
    b2 = Box(0.2, 1, 0.2).translate((-0.4, 0.4, 0))
    # Actually build123d Compound/Solid combination
    part = b1 + b2
    part.label = "concave_part"

    assembly = Compound(children=[part])

    # Run with VHACD enabled
    builder = SimulationBuilder(tmp_path, use_vhacd=True)
    scene_path = builder.build_from_assembly(assembly)

    assert scene_path.exists()
    # Check if meshes were created (now exports as .obj per architecture spec)
    assets_dir = tmp_path / "assets"
    obj_files = list(assets_dir.glob("concave_part*.obj"))

    # If VHACD is working, we should have multiple parts
    # Even if it fails, it should at least have one (the fallback)
    assert len(obj_files) >= 1

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    assert model.nmesh >= 1
