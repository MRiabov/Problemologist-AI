from collections.abc import Callable

from controller.agent.tools import get_common_tools
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from shared.simulation.schemas import SimulatorBackendType


def get_benchmark_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    common_tools = get_common_tools(fs, session_id)

    async def simulate(
        script_path: str, backend: SimulatorBackendType = SimulatorBackendType.GENESIS
    ) -> dict:
        """Run physics simulation for the benchmark.
        Genesis is used by default. MuJoCo can be selected for fast
        rigid-body only runs.
        """
        return await fs.simulate(script_path, backend=backend)

    async def validate(script_path: str):
        """Run geometric validation for the benchmark."""
        return await fs.validate(script_path)

    async def submit_for_review(script_path: str):
        """Submit the benchmark for review."""
        return await fs.submit(script_path)

    return [*common_tools, simulate, validate, submit_for_review]
