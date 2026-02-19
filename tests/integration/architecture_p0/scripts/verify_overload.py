import asyncio
import tempfile
from pathlib import Path

from shared.simulation.schemas import SimulatorBackendType
from worker.simulation.loop import SimulationLoop

# XML with position actuator and strict forcerange to trigger overload
TEST_OVERLOAD_XML = """
<mujoco>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="arm" pos="0 0 0.5">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <inertial pos="0 0 0" mass="10" diaginertia="1 1 1"/>
      <geom type="box" size=".2 .1 .1"/>
    </body>
  </worldbody>
  <actuator>
    <position name="servo" joint="hinge" kp="50" kv="5" forcerange="-0.1 0.1"/>
  </actuator>
</mujoco>
"""


async def run(_ctx=None):
    try:
        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as tmp:
            tmp.write(TEST_OVERLOAD_XML)
            tmp_path = Path(tmp.name)

        loop = SimulationLoop(str(tmp_path), backend_type=SimulatorBackendType.MUJOCO)

        # Demand large position that can't be reached with tiny forcerange
        # This will keep the motor saturated
        # We need to manually construct control inputs or let it drift?
        # worker.utils.controllers... might be available.

        # Simple constant control input:
        # data.ctrl[0] = 3.14

        # SimulationLoop.step applies control_inputs.
        # It expects a dict {actuator_name: value}.

        metrics = loop.step(control_inputs={"servo": 3.14}, duration=3.0)

        if not metrics.fail_reason or "motor_overload" not in metrics.fail_reason:
            raise RuntimeError(
                f"Expected motor_overload failure, but got: {metrics.fail_reason}"
            )

        return metrics.dict()
    except Exception:
        import traceback

        with open("debug_overload.txt", "w") as f:
            f.write(traceback.format_exc())
        raise
    finally:
        if "tmp_path" in locals() and tmp_path.exists():
            tmp_path.unlink()


if __name__ == "__main__":
    asyncio.run(run())
