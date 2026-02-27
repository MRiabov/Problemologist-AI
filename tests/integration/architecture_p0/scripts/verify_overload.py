import asyncio
import tempfile
from pathlib import Path

from worker_heavy.simulation.loop import SimulationLoop

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
    <position name="servo" joint="hinge" kp="100" kv="0" forcerange="-0.01 0.01"/>
  </actuator>
</mujoco>
"""


async def run(_ctx=None):
    try:
        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as tmp:
            tmp.write(TEST_OVERLOAD_XML)
            tmp_path = Path(tmp.name)

        from shared.simulation.schemas import SimulatorBackendType

        # Force MUJOCO to avoid GENESIS overhead/timeouts in CI/sandbox
        backend = SimulatorBackendType.MUJOCO

        loop = SimulationLoop(str(tmp_path), backend_type=backend)

        # Demand large position that can't be reached with tiny forcerange
        # This will keep the motor saturated
        metrics = loop.step(control_inputs={"servo": 100.0}, duration=5.0)

        from shared.enums import FailureReason

        if (
            not metrics.failure
            or metrics.failure.reason != FailureReason.MOTOR_OVERLOAD
        ):
            raise RuntimeError(
                f"Expected MOTOR_OVERLOAD failure, but got: {metrics.failure}"
            )

        return metrics.model_dump(mode="json")
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
