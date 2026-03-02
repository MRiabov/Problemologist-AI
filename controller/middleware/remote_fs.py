import logging
import os
from pathlib import Path
from typing import Literal

from temporalio.client import Client

from controller.clients.worker import WorkerClient
from controller.observability.middleware_helper import (
    broadcast_file_update,
    record_events,
    record_simulation_result,
)
from controller.workflows.execution import ScriptExecutionWorkflow
from controller.workflows.heavy import (
    HeavyPreviewWorkflow,
    HeavySimulationWorkflow,
    HeavyValidationWorkflow,
)
from shared.models.simulation import SimulationResult
from shared.observability.schemas import (
    EditFileToolEvent,
    GrepToolEvent,
    LibraryUsageEvent,
    LsFilesToolEvent,
    ManufacturabilityCheckEvent,
    PlanSubmissionEngineerEvent,
    ReadFileToolEvent,
    RunCommandToolEvent,
    SimulationRequestEvent,
    SkillReadEvent,
    WriteFileToolEvent,
)
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.filesystem.backend import FileInfo
from shared.workers.filesystem.policy import FilesystemPolicy
from shared.workers.schema import (
    BenchmarkToolResponse,
    EditOp,
    ExecuteResponse,
    GrepMatch,
    HeavyPreviewParams,
    HeavyPreviewResponse,
    HeavySimulationParams,
    HeavyValidationParams,
    HeavyValidationResponse,
    InspectTopologyResponse,
    PreviewDesignResponse,
    ScriptExecutionRequest,
)

logger = logging.getLogger(__name__)


# Global policy instance (cached)
_fs_policy: FilesystemPolicy | None = None


def get_fs_policy() -> FilesystemPolicy:
    global _fs_policy
    if _fs_policy is None:
        # Resolve config path relative to the project root or the current file.
        potential_paths = [
            Path("config/agents_config.yaml"),
            Path(__file__).parents[2] / "config" / "agents_config.yaml",
            Path("/app/config/agents_config.yaml"),
        ]

        config_path = next((p for p in potential_paths if p.exists()), None)

        if config_path is None:
            # Fallback for development if repo root is different
            logger.warning("agents_config_not_found_searching_repo")
            # Try finding it in the project root
            root = Path(__file__).parents[2]
            config_path = root / "config" / "agents_config.yaml"

        _fs_policy = FilesystemPolicy(config_path)
    return _fs_policy


class RemoteFilesystemMiddleware:
    """
    Middleware that proxies filesystem operations to a remote Worker,
    with durable execution via Temporal.
    """

    def __init__(
        self,
        client: WorkerClient,
        temporal_client: Client | None = None,
        agent_role: str = "engineering_mechanical_coder",
    ):
        self.client = client
        self.temporal_client = temporal_client
        self.agent_role = agent_role
        self.policy = get_fs_policy()

    def _check_perm(self, action: Literal["read", "write"], path: str | Path) -> None:
        """Check if action is allowed by policy."""
        if not self.policy.check_permission(self.agent_role, action, path):
            msg = f"Permission denied for {self.agent_role} to {action} '{path}'"
            logger.error(
                "filesystem_permission_denied",
                agent=self.agent_role,
                action=action,
                path=str(path),
            )
            raise PermissionError(msg)

    async def list_files(self, path: str | Path = "/") -> list[FileInfo]:
        """List files via the Worker client."""
        self._check_perm("read", path)
        await record_events(
            episode_id=self.client.session_id,
            events=[LsFilesToolEvent(path=str(path))],
        )
        return await self.client.list_files(str(path))

    async def glob_info(
        self, pattern: str, path: str | Path | None = None
    ) -> list[FileInfo]:
        """List files matching a glob pattern via the Worker client."""
        # Check permission for the base path
        base_path = path or "."
        self._check_perm("read", base_path)
        # We don't have a direct glob_info on worker, so we use grep with glob
        # but grep returns GrepMatch, not FileInfo.
        # Actually, let's just use list_files if path is provided, or implement glob on worker.
        # For now, we'll use grep to find files and then list_files for metadata if needed.
        # Alternatively, we can just use grep with a pattern that matches everything.
        matches = await self.client.grep(".*", path=str(base_path) if path else None, glob=pattern)
        # Convert GrepMatch to FileInfo (roughly)
        files = {}
        for m in matches:
            if m.path not in files:
                files[m.path] = FileInfo(
                    name=os.path.basename(m.path),
                    path=m.path,
                    type="file",
                    size=0, # Unknown without another call
                    mtime=0,
                )
        return list(files.values())

    async def upload_files(self, files: dict[str, str | bytes]) -> bool:
        """Upload multiple files to the worker."""
        for path, content in files.items():
            self._check_perm("write", path)
            if isinstance(content, str):
                content = content.encode("utf-8")
            await self.client.upload_file(path, content)
        return True

    async def download_files(self, paths: list[str]) -> dict[str, bytes]:
        """Download multiple files from the worker."""
        results = {}
        for path in paths:
            self._check_perm("read", path)
            results[path] = await self.client.read_file_binary(path)
        return results

    async def inspect_topology(
        self, target_id: str, script_path: str | Path = "script.py"
    ) -> InspectTopologyResponse:
        """Inspect topological features via the Worker client."""
        self._check_perm("read", script_path)
        # We could add a specific event type for this if needed
        return await self.client.inspect_topology(target_id, str(script_path))

    async def exists(self, path: str | Path) -> bool:
        """Check if a file exists via the Worker client."""
        self._check_perm("read", path)
        return await self.client.exists(str(path))

    async def read_file(self, path: str | Path) -> str:
        """Read file via the Worker client."""
        self._check_perm("read", path)
        p_str = str(path).lstrip("/")
        events = [ReadFileToolEvent(path=p_str)]
        if p_str.startswith("skills/"):
            skill_name = p_str.split("/")[1] if "/" in p_str[7:] else p_str[7:]
            # Simple heuristic for skill name
            events.append(SkillReadEvent(skill_path=path, skill_name=skill_name))

        await record_events(
            episode_id=self.client.session_id,
            events=events,
        )

        if p_str.startswith(("skills/", "utils/")):
            module_name = p_str.split("/")[1] if "/" in p_str[7:] else p_str[7:]
            await record_events(
                episode_id=self.client.session_id,
                events=[
                    LibraryUsageEvent(
                        module_name=module_name, usage_type="reused", path=p_str
                    )
                ],
            )

        return await self.client.read_file(str(path))

    async def write_file(
        self, path: str | Path, content: str, overwrite: bool = True
    ) -> bool:
        """Write file via the Worker client, enforcing read-only constraints."""
        self._check_perm("write", path)
        p_str = str(path)

        await record_events(
            episode_id=self.client.session_id,
            events=[
                WriteFileToolEvent(
                    path=p_str, content_snippet=content[:100], overwrite=overwrite
                )
            ],
        )
        success = await self.client.write_file(p_str, content, overwrite=overwrite)

        if success:
            # Track library usage (new)
            if path.lstrip("/").startswith(("skills/", "utils/")):
                module_name = (
                    path.lstrip("/").split("/")[1]
                    if "/" in path.lstrip("/")[7:]
                    else path.lstrip("/")[7:]
                )
                await record_events(
                    episode_id=self.client.session_id,
                    events=[
                        LibraryUsageEvent(
                            module_name=module_name, usage_type="new", path=path
                        )
                    ],
                )

            # Broadcast update and sync asset via helper
            await broadcast_file_update(self.client.session_id, p_str, content)

        return success

    async def edit_file(self, path: str | Path, edits: list[EditOp]) -> bool:
        """Edit a file via the Worker client."""
        self._check_perm("write", path)
        p_str = str(path)

        await record_events(
            episode_id=self.client.session_id,
            events=[EditFileToolEvent(path=p_str, num_edits=len(edits))],
        )
        success = await self.client.edit_file(p_str, edits)
        if success:
            # For edits, we read the file back to get the updated
            # content for the Asset table
            try:
                content = await self.client.read_file(p_str)
                await broadcast_file_update(self.client.session_id, p_str, content)
            except Exception:
                # Don't fail the edit if sync fails
                pass
        return success

    async def run_command(self, code: str, timeout: int = 30) -> ExecuteResponse:
        """
        Execute a command (Python code) via the Worker client,
        wrapped in Temporal for durability.
        """
        await record_events(
            episode_id=self.client.session_id,
            events=[RunCommandToolEvent(command=code)],
        )
        if self.temporal_client:
            # Wrap in Temporal workflow for durability
            return await self.temporal_client.execute_workflow(
                ScriptExecutionWorkflow.run,
                ScriptExecutionRequest(
                    code=code,
                    session_id=self.client.session_id,
                    timeout=timeout,
                ),
                id=f"exec-{self.client.session_id}-{hash(code) % 10**8}",
                task_queue="simulation-task-queue",
            )
        # Fallback to direct client call if Temporal is not available
        return await self.client.execute_python(code, timeout=timeout)

    # Adding alias for consistency with deepagents naming if needed
    async def execute(self, code: str, timeout: int = 30) -> ExecuteResponse:
        return await self.run_command(code, timeout=timeout)

    async def grep(
        self, pattern: str, path: str | Path | None = None, glob: str | None = None
    ) -> list[GrepMatch]:
        """Search for a pattern in files via the Worker client."""
        if path:
            self._check_perm("read", path)
        else:
            self._check_perm("read", ".")

        p_str = str(path) if path else None
        await record_events(
            episode_id=self.client.session_id,
            events=[GrepToolEvent(pattern=pattern, path=p_str, glob=glob)],
        )
        return await self.client.grep(pattern, path=p_str, glob=glob)

    async def simulate(
        self,
        script_path: str | Path,
        backend: SimulatorBackendType = SimulatorBackendType.GENESIS,
    ) -> BenchmarkToolResponse | SimulationResult:
        """Trigger physics simulation via worker client (with bundling)."""
        p_str = str(script_path)
        # Record request
        await record_events(
            episode_id=self.client.session_id,
            events=[SimulationRequestEvent(script_path=p_str)],
        )

        if self.temporal_client:
            # Bundle from light worker
            bundle = await self.client.bundle_session()

            # Execute via Temporal
            res = await self.temporal_client.execute_workflow(
                HeavySimulationWorkflow.run,
                HeavySimulationParams(
                    bundle_bytes=bundle,
                    script_path=p_str,
                    backend=backend,
                    smoke_test_mode=False,
                    session_id=self.client.session_id,
                ),
                id=f"sim-{self.client.session_id}-{abs(hash(p_str)) % 10**8}",
                task_queue="simulation-task-queue",
            )
            await record_simulation_result(self.client.session_id, res)
            return res

        return await self.client.simulate(p_str, backend=backend)

    async def preview(
        self,
        script_path: str | Path,
        pitch: float = -45.0,
        yaw: float = 45.0,
    ) -> PreviewDesignResponse:
        """Trigger design preview via worker client (with bundling)."""
        p_str = str(script_path)

        if self.temporal_client:
            bundle = await self.client.bundle_session()
            res: HeavyPreviewResponse = await self.temporal_client.execute_workflow(
                HeavyPreviewWorkflow.run,
                HeavyPreviewParams(
                    bundle_bytes=bundle,
                    script_path=p_str,
                    pitch=pitch,
                    yaw=yaw,
                ),
                id=f"prev-{self.client.session_id}-{abs(hash(p_str)) % 10**8}",
                task_queue="simulation-task-queue",
            )
            return PreviewDesignResponse(
                success=res.success,
                message="Preview generated via Temporal"
                if res.success
                else "Preview failed",
                image_path=res.filename,
            )

        return await self.client.preview(p_str, pitch=pitch, yaw=yaw)

    async def validate(
        self, script_path: str | Path
    ) -> BenchmarkToolResponse | HeavyValidationResponse:
        """Trigger geometric validation via worker client (with bundling)."""
        p_str = str(script_path)

        if self.temporal_client:
            bundle = await self.client.bundle_session()
            res = await self.temporal_client.execute_workflow(
                HeavyValidationWorkflow.run,
                HeavyValidationParams(
                    bundle_bytes=bundle,
                    script_path=p_str,
                    session_id=self.client.session_id,
                ),
                id=f"val-{self.client.session_id}-{abs(hash(p_str)) % 10**8}",
                task_queue="simulation-task-queue",
            )
        else:
            res = await self.client.validate(p_str)

        await record_events(
            episode_id=self.client.session_id,
            events=[
                ManufacturabilityCheckEvent(
                    part_id=p_str,  # Using script_path as part identifier
                    method="auto_geometric",
                    result=res.success,
                    price=getattr(res, "price", None),
                    weight_g=getattr(res, "weight_g", None),
                    metadata=res.model_dump(),
                )
            ],
        )

        return res

    async def submit(self, script_path: str | Path) -> BenchmarkToolResponse:
        """Trigger handover to review via worker client."""
        p_str = str(script_path)
        res = await self.client.submit(p_str)

        # Record as PlanSubmissionEngineerEvent
        await record_events(
            episode_id=self.client.session_id,
            events=[
                PlanSubmissionEngineerEvent(
                    plan_path=p_str,
                )
            ],
        )

        return res
