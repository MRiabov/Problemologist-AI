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
    # Use a directory inside the project so it's mounted in the sandbox
    project_root = Path(__file__).resolve().parent.parent.parent
    asset_dir = project_root / "test_assets"
    asset_dir.mkdir(exist_ok=True)

    try:
        provider = BDWarehouseProvider()
        # Nema17 has id bd_warehouse:motor:Nema17
        part = provider.get_part("bd_warehouse:motor:Nema17")
        motor_compound = part.factory()

        # Create a simple agent compound
        agent = motor_compound
        agent.label = "motor_p0"

        # Compile
        compiler = SceneCompiler(asset_dir=str(asset_dir))
        xml_str = compiler.compile(Compound([Box(0.1, 0.1, 0.1)]), agent)

        print(f"Generated XML snippet: {xml_str[:500]}...")

        # Adjust paths for sandbox
        xml_str = xml_str.replace(str(asset_dir), "/workspace/test_assets")

        # Verify XML contains expected elements
        assert "joint" in xml_str
        assert "motor" in xml_str
        assert "stator" in xml_str
        assert "rotor" in xml_str

        # Save and load
        xml_path = asset_dir / "test_motor.xml"
        xml_path.write_text(xml_str)

        model = mujoco.MjModel.from_xml_path(str(xml_path))
        assert model.njnt >= 1
        assert model.nu >= 1

        # Run simulation loop to check if it moves
        sim = SimulationLoop(str(xml_path))

        # Apply control to the motor
        # If model.nu is 1, return [1.0]
        agent_script = """
def control(obs):
    import numpy as np
    return [1.0]
"""
        result = sim.run(agent_script, max_steps=50)
        print(f"DEBUG: Simulation result: {result}")

        # Verify success/timeout
        assert result["status"] in ["TIMEOUT", "RUNNING"]
        # Verify energy was consumed (implies movement/torque)
        assert result["metrics"]["energy"] > 0
        print(
            f"Simulation result: {result['status']}, Energy: {result['metrics']['energy']}"
        )
    finally:
        import shutil

        if asset_dir.exists():
            shutil.rmtree(asset_dir)


if __name__ == "__main__":
    test_nema17_automatic_joint()
