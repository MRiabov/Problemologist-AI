import textwrap

import pytest

from controller.agent.benchmark.nodes import verify_syntax
from worker.utils.validation import simulate, validate


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
            
            part = p.part
            part.label = "target_box"
            
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
    is_geom_valid = validate(component)
    assert is_geom_valid is True

    # 4. Simulate without crashing
    # This uses worker.utils.validation.simulate which runs MuJoCo for a few frames
    # and generates renders.
    sim_result = simulate(component)

    assert sim_result.success is True
    assert "Simulation stable" in sim_result.summary
    # verify it actually did something (e.g., render paths populated)
    # Note: prerender_24_views might be slow or require GL context
    # If it fails due to headless environment, we might need to mock it
    # but the goal is to test the integration.
