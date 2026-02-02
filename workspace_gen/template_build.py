from build123d import *
from src.simulation_engine.builder import SceneCompiler


def build(seed):
    return f'<mujoco><worldbody><geom name="box_{seed}" type="box" size="1 1 1"/></worldbody></mujoco>'
