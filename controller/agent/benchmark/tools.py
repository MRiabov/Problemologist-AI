from collections.abc import Callable

from controller.agent.tools import get_common_tools
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from shared.enums import AgentName
from shared.models.schemas import PlannerSubmissionResult
from shared.simulation.schemas import SimulatorBackendType


def get_benchmark_planner_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """Planner-only filesystem toolset to keep planning loops focused."""

    async def list_files(path: str = "/"):
        return await fs.list_files(path)

    async def read_file(path: str):
        return await fs.read_file(path)

    async def write_file(path: str, content: str, overwrite: bool = False):
        return await fs.write_file(path, content, overwrite=overwrite)

    async def edit_file(path: str, old_string: str, new_string: str):
        from controller.middleware.remote_fs import EditOp

        return await fs.edit_file(
            path, [EditOp(old_string=old_string, new_string=new_string)]
        )

    async def grep(pattern: str, path: str | None = None, glob: str | None = None):
        return await fs.grep(pattern, path, glob)

    async def submit_plan() -> dict:
        """
        Validate benchmark planner artifacts and explicitly submit planner handoff.

        Returns:
            {"ok": bool, "status": "submitted"|"rejected", "errors": [...], "node_type": "..."}
        """
        from worker_heavy.utils.file_validation import validate_node_output

        required_files = ["plan.md", "todo.md", "objectives.yaml"]
        artifacts: dict[str, str] = {}
        missing_files: list[str] = []

        for rel_path in required_files:
            if not await fs.exists(rel_path):
                missing_files.append(rel_path)
                continue
            content = await fs.read_file(rel_path)
            if not content.strip():
                missing_files.append(rel_path)
                continue
            artifacts[rel_path] = content

        if missing_files:
            result = PlannerSubmissionResult(
                ok=False,
                status="rejected",
                errors=[f"Missing required file: {p}" for p in missing_files],
                node_type=AgentName.BENCHMARK_PLANNER,
            )
            return result.model_dump(mode="json")

        is_valid, errors = validate_node_output(AgentName.BENCHMARK_PLANNER, artifacts)
        result = PlannerSubmissionResult(
            ok=is_valid,
            status="submitted" if is_valid else "rejected",
            errors=errors,
            node_type=AgentName.BENCHMARK_PLANNER,
        )
        return result.model_dump(mode="json")

    return [list_files, read_file, write_file, edit_file, grep, submit_plan]


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
