import textwrap
from unittest.mock import patch

import numpy as np
import pytest

pytestmark = [pytest.mark.integration, pytest.mark.xdist_group(name="physics_sims")]


def verify_syntax(script: str) -> tuple[bool, str | None]:
    try:
        compile(script, "<string>", "exec")
        return True, None
    except Exception as e:
        return False, str(e)


from worker_heavy.utils.validation import simulate, validate


def test_model_produced_script_integration(tmp_path):
    """
    Integration Test:
    Verifies that a script typically produced by the model:
    1. Passes syntax verification.
    2. Can be executed to produce a build123d Compound and MJCF string.
    3. The produced geometry can be validated.
    4. The produced geometry can be simulated without crashing.
    """

    # This is a representative script that CAD Coder agent might produce
    model_script = textwrap.dedent("""
        from build123d import *
        import mujoco
        
        def build(seed: int = 0, scale: float = 1.0):
            # Use seed for some variation
            width = 0.1 + (seed % 10) / 100.0
            
            with BuildPart() as p:
                with BuildSketch():
                    Rectangle(width, 0.1)
                extrude(amount=0.05)
            
            from shared.models.schemas import PartMetadata
            from shared.enums import ManufacturingMethod
            part = p.part
            part.label = "target_box"
            part.metadata = PartMetadata(
                material_id="aluminum_6061",
                manufacturing_method=ManufacturingMethod.THREE_DP
            )
            
            # Simple MJCF that uses the mesh
            mjcf_xml = f'''
            <mujoco model="generated_scene">
              <worldbody>
                <light pos="0 0 2"/>
                <geom name="floor" type="plane" size="1 1 0.1"/>
                <body name="target_body" pos="0 0 0.5">
                  <freejoint/>
                  <geom name="target_mesh" type="box" size="{width/2} 0.05 0.025" mass="1"/>
                </body>
              </worldbody>
            </mujoco>
            '''
            return part, mjcf_xml
    """).strip()

    # 1. Verify Syntax
    is_valid, error = verify_syntax(model_script)
    assert is_valid, f"Syntax verification failed: {error}"

    # 2. Execute and Produce Geometry
    local_scope = {}
    try:
        exec(model_script, local_scope)
    except Exception as e:
        pytest.fail(f"Execution of model script failed: {e}")

    assert "build" in local_scope
    build_func = local_scope["build"]

    component, mjcf = build_func(seed=42)
    assert component is not None
    assert isinstance(mjcf, str)
    assert "<mujoco" in mjcf

    # 3. Validate Geometry
    # This uses our worker.utils.validation.validate which checks for intersections/bounds
    is_geom_valid = validate(component, output_dir=tmp_path)
    assert is_geom_valid[0] is True

    # 4. Simulate without crashing
    # Mock mujoco.Renderer to avoid framebuffer issues in headless environment
    with patch("mujoco.Renderer") as mock_renderer_cls:
        mock_renderer = mock_renderer_cls.return_value
        mock_renderer.render.return_value = np.zeros((480, 640, 3), dtype=np.uint8)

        sim_result = simulate(component, output_dir=tmp_path, smoke_test_mode=True)

    assert sim_result.success is True
    assert any(msg in sim_result.summary for msg in ["Simulation stable", "Goal achieved"])
