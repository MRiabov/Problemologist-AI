import xml.etree.ElementTree as ET
import mujoco
from build123d import Box, Compound, BuildPart
from src.simulation_engine.builder import SceneCompiler

def test_mjcf_compilation():
    compiler = SceneCompiler()
    
    with BuildPart() as b:
        Box(10, 10, 10)
    
    env_c = b.part
    
    xml_string = compiler.compile(env_c)
    print("Generated XML:")
    print(xml_string[:500])
    
    try:
        model = mujoco.MjModel.from_xml_string(xml_string)
        print("MuJoCo successfully loaded the model!")
    except Exception as e:
        print(f"MuJoCo failed to load the model: {e}")

if __name__ == "__main__":
    test_mjcf_compilation()