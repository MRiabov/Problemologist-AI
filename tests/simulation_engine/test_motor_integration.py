import tempfile
import sys
from pathlib import Path
import mujoco
from build123d import Compound, Box
from src.cots.providers.bd_warehouse import BDWarehouseProvider
from src.simulation_engine.builder import SceneCompiler
from src.simulation_engine.simulation import SimulationLoop

print(f"DEBUG: sys.path = {sys.path}")
import src.simulation_engine.builder

print(f"DEBUG: SceneCompiler from {src.simulation_engine.builder.__file__}")


def test_nema17_automatic_joint():
    from src.environment import tools
    # Ensure we are in a clean state in the workspace
    workspace = Path(tools.WORKSPACE_DIR).resolve()
    # Use a relative path for asset_dir inside the workspace
    # We will run the test from the workspace directory conceptually
    asset_rel_path = "test_assets"
    asset_dir = workspace / asset_rel_path
    if asset_dir.exists():
        import shutil
        shutil.rmtree(asset_dir)
    asset_dir.mkdir(parents=True, exist_ok=True)

    try:
        provider = BDWarehouseProvider()
        # Nema17 has id bd_warehouse:motor:Nema17
        part = provider.get_part("bd_warehouse:motor:Nema17")
        motor_compound = part.factory()

        # Create a simple agent compound
        agent = motor_compound
        agent.label = "motor_p0"

        # Compile with a RELATIVE path so the XML is portable
        compiler = SceneCompiler(asset_dir=asset_rel_path)
        
        # We need to be in the workspace for relative paths to work during compilation
        # because MeshProcessor writes to the filesystem.
        import os
        old_cwd = os.getcwd()
        os.chdir(workspace)
        
        try:
            xml_str = compiler.compile(Compound([Box(0.1, 0.1, 0.1)]), agent)
            
            # Save XML in the workspace
            xml_path = workspace / "test_motor.xml"
            xml_path.write_text(xml_str)
            
            # Verify XML contains expected elements
            assert "joint" in xml_str
            assert "motor" in xml_str
            assert "stator" in xml_str
            assert "rotor" in xml_str
            
            # Local load should work now because meshdir="test_assets" 
            # and we are in the workspace
            model = mujoco.MjModel.from_xml_path(str(xml_path))
            assert model.njnt >= 1
            assert model.nu >= 1

            # Run simulation loop
            sim = SimulationLoop(str(xml_path))
            
            agent_script = """
def control(obs):
    import numpy as np
    return [1.0]
"""
            result = sim.run(agent_script, max_steps=50)
            print(f"DEBUG: Simulation result: {result}")
            
            assert result["status"] in ["TIMEOUT", "RUNNING", "WIN"]
            assert result["metrics"]["energy"] > 0
        finally:
            os.chdir(old_cwd)
    finally:
        import shutil
        if asset_dir.exists():
            shutil.rmtree(asset_dir)


if __name__ == "__main__":
    test_nema17_automatic_joint()
