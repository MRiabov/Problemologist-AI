import os
import pytest
import numpy as np
from shared.simulation.backends import SimulatorBackendType
from worker.simulation.loop import SimulationLoop

PARITY_XML = """<mujoco model="parity_test">
    <option gravity="0 0 -9.81" timestep="0.002"/>
    <worldbody>
        <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
        <geom name="floor" type="plane" size="10 10 0.1" rgba=".8 .8 .8 1"/>
        
        <!-- Falling Box -->
        <body name="box" pos="0 0 1">
            <joint name="box_free" type="free"/>
            <geom name="box_geom" type="box" size="0.1 0.1 0.1" rgba="0 1 0 1" mass="1"/>
        </body>

        <!-- Hinge Pendulum -->
        <body name="pendulum_link" pos="1 0 1">
            <joint name="pendulum_hinge" type="hinge" axis="0 1 0"/>
            <geom name="pendulum_geom" type="capsule" fromto="0 0 0 0.5 0 0" size="0.05" rgba="1 0 0 1" mass="1"/>
        </body>
    </worldbody>
</mujoco>"""


@pytest.fixture
def parity_xml_path(tmp_path):
    path = tmp_path / "parity_scene.xml"
    path.write_text(PARITY_XML)
    return str(path)


@pytest.mark.integration
def test_physics_parity_mujoco_genesis(parity_xml_path):
    # Skip if Genesis is not installed or failing to init
    try:
        import genesis as gs
    except ImportError:
        pytest.skip("Genesis not installed")

    # 1. Run MuJoCo
    loop_mj = SimulationLoop(parity_xml_path, backend_type=SimulatorBackendType.MUJOCO)
    # Run for 0.5 seconds
    metrics_mj = loop_mj.step(control_inputs={}, duration=0.5)

    pos_box_mj = loop_mj.backend.get_body_state("box").pos
    pos_pend_mj = loop_mj.backend.get_body_state("pendulum_link").pos

    # 2. Run Genesis
    loop_gs = SimulationLoop(parity_xml_path, backend_type=SimulatorBackendType.GENESIS)
    metrics_gs = loop_gs.step(control_inputs={}, duration=0.5)

    pos_box_gs = loop_gs.backend.get_body_state("box").pos
    pos_pend_gs = loop_gs.backend.get_body_state("pendulum_link").pos

    # 3. Compare Box Position (falling)
    pos_box_mj = np.array(pos_box_mj)
    pos_box_gs = np.array(pos_box_gs)

    dist_box = np.linalg.norm(pos_box_mj - pos_box_gs)
    print(f"Box MJ: {pos_box_mj}, GS: {pos_box_gs}, Diff: {dist_box}")

    # 4. Compare Pendulum Position (swinging)
    pos_pend_mj = np.array(pos_pend_mj)
    pos_pend_gs = np.array(pos_pend_gs)
    dist_pend = np.linalg.norm(pos_pend_mj - pos_pend_gs)
    print(f"Pendulum MJ: {pos_pend_mj}, GS: {pos_pend_gs}, Diff: {dist_pend}")

    # Tolerance: < 5% difference relative to typical scale (1.0m)
    assert dist_box < 0.05, f"Box position mismatch: {dist_box}"
    assert dist_pend < 0.05, f"Pendulum position mismatch: {dist_pend}"


if __name__ == "__main__":
    pytest.main([__file__])
