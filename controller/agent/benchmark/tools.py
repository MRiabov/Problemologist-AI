from collections.abc import Callable

from controller.agent.tools import get_common_tools
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from shared.simulation.schemas import SimulatorBackendType


def get_benchmark_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """Base tools for benchmark generator agents."""
    # We remove validate_costing_and_price if it's there by any chance
    # but get_common_tools doesn't have it anymore.
    return get_common_tools(fs, session_id)


def get_benchmark_planner_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """Tools for the Benchmark Planner."""
    from controller.agent.tools import get_planner_tools

    return get_planner_tools(fs, session_id)


def get_benchmark_coder_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """Tools for the Benchmark Coder/Implementer."""
    from controller.agent.tools import get_coder_tools

    base_tools = get_coder_tools(fs, session_id)

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

    return [*base_tools, simulate, validate, submit_for_review]
