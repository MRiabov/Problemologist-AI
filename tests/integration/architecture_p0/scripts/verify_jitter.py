import asyncio
import tempfile
from pathlib import Path

from worker.simulation.verification import verify_with_jitter

# Create a dummy XML that always succeeds
TEST_SUCCESS_XML = """
<mujoco>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="target_box" pos="0 0 0.5">
      <joint type="free"/>
      <geom name="ball" type="sphere" size="0.05"/>
    </body>
    <geom name="floor" type="plane" pos="0 0 0" size="10 10 0.1"/>
    <site name="zone_goal" type="box" pos="0 0 0.05" size="0.5 0.5 0.1" rgba="0 1 0 0.3"/>
  </worldbody>
</mujoco>
"""


async def run(_ctx=None):
    # This script simulates what an agent might do to verify their design
    # It writes a temp XLM and calls verification
    try:
        with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", delete=False) as tmp:
            tmp.write(TEST_SUCCESS_XML)
            tmp_path = Path(tmp.name)

        result = verify_with_jitter(
            xml_path=str(tmp_path),
            control_inputs={},
            num_runs=3,
            duration=1.0,
            seed=42,
        )
        print(
            f"VERIFICATION_RESULT: success_rate={result.success_rate}, "
            f"consistent={result.is_consistent}"
        )
        return result.dict()
    except Exception as e:
        import traceback

        with open("debug_jitter.txt", "w") as f:
            f.write(traceback.format_exc())
        print(traceback.format_exc())
        raise
    finally:
        if "tmp_path" in locals() and tmp_path.exists():
            tmp_path.unlink()


if __name__ == "__main__":
    asyncio.run(run())
