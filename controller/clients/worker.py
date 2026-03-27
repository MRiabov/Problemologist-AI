import asyncio
import base64
import os
from typing import Any, Literal

import httpx
import structlog

from controller.config.settings import settings
from shared.enums import AgentName, ResponseStatus
from shared.simulation.schemas import (
    SimulatorBackendType,
    get_default_simulator_backend,
)
from shared.workers.filesystem.backend import FileInfo
from shared.workers.schema import (
    BenchmarkToolResponse,
    EditOp,
    ExecuteResponse,
    GitCommitResponse,
    GitStatusResponse,
    GrepMatch,
    InspectTopologyResponse,
    ReviewerStage,
)
from shared.workers.workbench_models import ManufacturingMethod, WorkbenchResult

logger = structlog.get_logger(__name__)


def _default_smoke_test_mode() -> bool:
    env_smoke = os.getenv("SMOKE_TEST_MODE")
    if env_smoke is not None:
        return env_smoke.strip().lower() in {"1", "true", "yes", "on"}
    return settings.is_integration_test


class WorkerClient:
    """Async client for interacting with the Problemologist Worker API."""

    def __init__(
        self,
        base_url: str,
        session_id: str,
        http_client: httpx.AsyncClient | None = None,
        heavy_url: str | None = None,
        controller_url: str | None = None,
        agent_role: AgentName | str = AgentName.ENGINEER_CODER,
    ):
        """Initialize the worker client.

        Args:
            base_url: Base URL of the light worker service (e.g., http://worker-light:8000).
            session_id: Session ID to use for all requests.
            http_client: Pre-configured httpx.AsyncClient to reuse.
            heavy_url: Optional base URL of the heavy worker service.
        """
        self.base_url = base_url.rstrip("/")
        self.light_url = self.base_url
        self.heavy_url = (heavy_url or base_url).rstrip("/")
        controller_base_url = (
            controller_url
            if controller_url is not None
            else os.getenv("CONTROLLER_URL")
        )
        self.controller_url = (
            controller_base_url.rstrip("/") if controller_base_url else None
        )
        self.session_id = session_id
        self.agent_role = (
            agent_role.value if isinstance(agent_role, AgentName) else str(agent_role)
        )
        self.headers = {
            "X-Session-ID": session_id,
            "X-Agent-Role": self.agent_role,
            "X-Stage": self.agent_role,
        }
        self.http_client = http_client
        self._loop_clients: dict[int, httpx.AsyncClient] = {}
        self._loop_locks: dict[int, asyncio.Lock] = {}

    def _request_headers(
        self,
        *,
        bypass_agent_permissions: bool = False,
        stage: str | None = None,
        reviewer_stage: ReviewerStage | None = None,
    ) -> dict[str, str]:
        headers = dict(self.headers)
        if stage:
            headers["X-Stage"] = stage
        if reviewer_stage:
            headers["X-Reviewer-Stage"] = reviewer_stage
            headers["X-Stage"] = reviewer_stage
        if bypass_agent_permissions:
            headers["X-System-FS-Bypass"] = "1"
        return headers

    async def _get_client(self) -> httpx.AsyncClient:
        """Helper to get a client, either shared or new (for compatibility)."""
        if self.http_client:
            return self.http_client

        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            # Fallback for sync contexts that might somehow call this
            return httpx.AsyncClient()

        loop_id = id(loop)

        if loop_id in self._loop_clients:
            client = self._loop_clients[loop_id]
            # Check if the client is effectively closed or the loop is not running
            if getattr(client, "is_closed", False) or not loop.is_running():
                del self._loop_clients[loop_id]

        if loop_id not in self._loop_clients:
            if loop_id not in self._loop_locks:
                self._loop_locks[loop_id] = asyncio.Lock()

            async with self._loop_locks[loop_id]:
                if loop_id not in self._loop_clients:
                    self._loop_clients[loop_id] = httpx.AsyncClient()

        return self._loop_clients[loop_id]

    async def _close_client(self, client: httpx.AsyncClient):
        """Helper to close a client only if it was created locally."""
        # We no longer close the client here to allow connection pooling across requests.
        # The client will be closed when aclose() is called or the instance is destroyed.
        pass

    @staticmethod
    def _default_reviewer_stage(agent_role: AgentName | str) -> ReviewerStage | None:
        role_value = (
            agent_role.value if isinstance(agent_role, AgentName) else str(agent_role)
        )
        role_to_stage: dict[str, ReviewerStage] = {
            AgentName.BENCHMARK_CODER.value: "benchmark_reviewer",
            AgentName.BENCHMARK_REVIEWER.value: "benchmark_reviewer",
            AgentName.ELECTRONICS_REVIEWER.value: "electronics_reviewer",
            AgentName.ENGINEER_CODER.value: "engineering_execution_reviewer",
            AgentName.ENGINEER_EXECUTION_REVIEWER.value: "engineering_execution_reviewer",
        }
        return role_to_stage.get(role_value)

    async def _call_controller_script_tool(
        self, action: str, payload: dict[str, Any], *, stage: str | None = None
    ) -> BenchmarkToolResponse | None:
        if not self.controller_url:
            return None
        client = await self._get_client()
        response = await client.post(
            f"{self.controller_url}/api/script-tools/{action.lstrip('/')}",
            json=payload,
            headers=self._request_headers(stage=stage),
            timeout=1000.0,
        )
        response.raise_for_status()
        return BenchmarkToolResponse.model_validate(response.json())

    async def aclose(self):
        """Explicitly close all HTTP clients created for different loops."""
        # WP08: Handle loop-specific clients
        for loop_id, client in list(self._loop_clients.items()):
            await client.aclose()
            del self._loop_clients[loop_id]

        # Compatibility for legacy _cached_client check in tests
        if hasattr(self, "_cached_client") and self._cached_client:
            await self._cached_client.aclose()
            self._cached_client = None

    async def inspect_topology(
        self, target_id: str, script_path: str = "script.py"
    ) -> InspectTopologyResponse:
        """Inspect topological features via worker."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/topology/inspect",
                json={"target_id": target_id, "script_path": script_path},
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return InspectTopologyResponse.model_validate(response.json())
        finally:
            await self._close_client(client)

    async def list_files(
        self, path: str = "/", *, bypass_agent_permissions: bool = False
    ) -> list[FileInfo]:
        """List contents of a directory."""
        if not path:
            path = "/"
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/ls",
                json={
                    "path": path,
                    "bypass_agent_permissions": bypass_agent_permissions,
                },
                headers=self._request_headers(
                    bypass_agent_permissions=bypass_agent_permissions
                ),
                timeout=10.0,
            )
            response.raise_for_status()
            return [FileInfo.model_validate(item) for item in response.json()]
        finally:
            await self._close_client(client)

    async def grep(
        self,
        pattern: str,
        path: str | None = None,
        glob: str | None = None,
        *,
        bypass_agent_permissions: bool = False,
    ) -> list[GrepMatch]:
        """Search for a pattern in files."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/grep",
                json={
                    "pattern": pattern,
                    "path": path,
                    "glob": glob,
                    "bypass_agent_permissions": bypass_agent_permissions,
                },
                headers=self._request_headers(
                    bypass_agent_permissions=bypass_agent_permissions
                ),
                timeout=30.0,
            )
            response.raise_for_status()
            return [GrepMatch.model_validate(item) for item in response.json()]
        finally:
            await self._close_client(client)

    async def read_file(
        self, path: str, *, bypass_agent_permissions: bool = False
    ) -> str:
        """Read file contents."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/read",
                json={
                    "path": path,
                    "bypass_agent_permissions": bypass_agent_permissions,
                },
                headers=self._request_headers(
                    bypass_agent_permissions=bypass_agent_permissions
                ),
                timeout=10.0,
            )
            if response.status_code == 404:
                return f"Error: File '{path}' not found."
            response.raise_for_status()
            return response.json()["content"]
        finally:
            await self._close_client(client)

    async def read_file_optional(
        self, path: str, *, bypass_agent_permissions: bool = False
    ) -> str | None:
        """Read file contents and return ``None`` when the file does not exist."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/read",
                json={
                    "path": path,
                    "bypass_agent_permissions": bypass_agent_permissions,
                },
                headers=self._request_headers(
                    bypass_agent_permissions=bypass_agent_permissions
                ),
                timeout=10.0,
            )
            if response.status_code == 404:
                return None
            response.raise_for_status()
            return response.json()["content"]
        finally:
            await self._close_client(client)

    async def exists(
        self, path: str, *, bypass_agent_permissions: bool = False
    ) -> bool:
        """Check if a file exists."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/exists",
                json={
                    "path": path,
                    "bypass_agent_permissions": bypass_agent_permissions,
                },
                headers=self._request_headers(
                    bypass_agent_permissions=bypass_agent_permissions
                ),
                timeout=10.0,
            )
            if response.status_code == 404:
                return False
            response.raise_for_status()
            return response.json()["exists"]
        except Exception:
            return False
        finally:
            await self._close_client(client)

    async def write_file(
        self,
        path: str,
        content: str,
        overwrite: bool = True,
        *,
        bypass_agent_permissions: bool = False,
    ) -> bool:
        """Write content to a file."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/write",
                json={
                    "path": path,
                    "content": content,
                    "overwrite": overwrite,
                    "bypass_agent_permissions": bypass_agent_permissions,
                },
                headers=self._request_headers(
                    bypass_agent_permissions=bypass_agent_permissions
                ),
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == ResponseStatus.SUCCESS
        finally:
            await self._close_client(client)

    async def bundle_session(self) -> bytes:
        """Bundle the session workspace into a gzipped tarball from the light worker."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.light_url}/fs/bundle",
                headers=self.headers,
                timeout=60.0,
            )
            response.raise_for_status()
            return response.content
        finally:
            await self._close_client(client)

    async def _add_bundle_to_payload(self, payload: dict):
        """Append workspace bundle payload for split light/heavy worker mode."""
        if self.light_url != self.heavy_url:
            bundle = await self.bundle_session()
            payload["bundle_base64"] = base64.b64encode(bundle).decode("utf-8")

    async def _sync_handover_artifacts_to_light(
        self,
        response: BenchmarkToolResponse,
    ) -> None:
        """
        Mirror critical handover artifacts returned by the heavy worker into the
        light-worker workspace when running split-worker mode.
        """
        if self.light_url == self.heavy_url or not response.artifacts:
            return

        artifacts = response.artifacts
        writes: list[tuple[str, str]] = []
        if artifacts.validation_results_json:
            writes.append(
                ("validation_results.json", artifacts.validation_results_json)
            )
        if artifacts.simulation_result_json:
            writes.append(("simulation_result.json", artifacts.simulation_result_json))
        if artifacts.review_manifests_json:
            writes.extend(
                [
                    (path, content)
                    for path, content in artifacts.review_manifests_json.items()
                ]
            )
        elif artifacts.review_manifest_json:
            # Backward compatibility: keep honoring single-manifest payloads.
            writes.append(
                (
                    ".manifests/engineering_execution_review_manifest.json",
                    artifacts.review_manifest_json,
                )
            )

        for path, content in writes:
            ok = await self.write_file(
                path,
                content,
                overwrite=True,
                bypass_agent_permissions=True,
            )
            if not ok:
                logger.warning("handover_artifact_sync_write_failed", path=path)
            else:
                logger.info("handover_artifact_synced", path=path)

        for path, encoded in artifacts.render_blobs_base64.items():
            ok = await self.upload_file(
                path,
                base64.b64decode(encoded),
                bypass_agent_permissions=True,
            )
            if not ok:
                logger.warning("handover_render_sync_upload_failed", path=path)
            else:
                logger.info("handover_render_synced", path=path)

    async def upload_file(
        self,
        path: str,
        content: bytes,
        *,
        bypass_agent_permissions: bool = False,
    ) -> bool:
        """Upload a file with binary content."""
        client = await self._get_client()
        try:
            # Prepare multipart form data
            files = {"file": ("blob", content)}
            data = {
                "path": path,
                "bypass_agent_permissions": str(bypass_agent_permissions).lower(),
            }
            response = await client.post(
                f"{self.base_url}/fs/upload_file",
                data=data,
                files=files,
                headers=self._request_headers(
                    bypass_agent_permissions=bypass_agent_permissions
                ),
                timeout=30.0,
            )
            response.raise_for_status()
            return response.json()["status"] == ResponseStatus.SUCCESS
        finally:
            await self._close_client(client)

    async def read_file_binary(
        self, path: str, *, bypass_agent_permissions: bool = False
    ) -> bytes:
        """Read file contents as binary."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/fs/read_blob",
                json={
                    "path": path,
                    "bypass_agent_permissions": bypass_agent_permissions,
                },
                headers=self._request_headers(
                    bypass_agent_permissions=bypass_agent_permissions
                ),
                timeout=30.0,
            )
            response.raise_for_status()
            return response.content
        finally:
            await self._close_client(client)

    async def preview(
        self,
        script_path: str = "script.py",
        script_content: str | None = None,
        pitch: float = -45.0,
        yaw: float = 45.0,
    ) -> dict[str, Any]:
        """Trigger design preview via worker."""
        client = await self._get_client()
        try:
            payload = {
                "script_path": script_path,
                "pitch": pitch,
                "yaw": yaw,
            }
            if script_content is not None:
                payload["script_content"] = script_content

            await self._add_bundle_to_payload(payload)

            response = await client.post(
                f"{self.heavy_url}/benchmark/preview",
                json=payload,
                headers=self._request_headers(stage=self.agent_role),
                timeout=30.0,
            )
            response.raise_for_status()
            return response.json()
        finally:
            await self._close_client(client)

    async def edit_file(
        self,
        path: str,
        edits: list[EditOp],
        *,
        bypass_agent_permissions: bool = False,
    ) -> bool:
        """Edit a file with one or more operations."""
        client = await self._get_client()
        try:
            json_edits = [op.model_dump() for op in edits]
            response = await client.post(
                f"{self.base_url}/fs/edit",
                json={
                    "path": path,
                    "edits": json_edits,
                    "bypass_agent_permissions": bypass_agent_permissions,
                },
                headers=self._request_headers(
                    bypass_agent_permissions=bypass_agent_permissions
                ),
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == ResponseStatus.SUCCESS
        finally:
            await self._close_client(client)

    async def execute_command(
        self,
        command: str,
        timeout: int = 30,
        *,
        episode_id: str | None = None,
    ) -> ExecuteResponse:
        """Execute a shell command in the session runtime."""
        client = await self._get_client()
        try:
            http_timeout = float(timeout) + 5.0
            response = await client.post(
                f"{self.base_url}/runtime/execute",
                json={
                    "code": command,
                    "timeout": timeout,
                    "episode_id": episode_id,
                },
                headers=self._request_headers(stage=self.agent_role),
                timeout=http_timeout,
            )
            response.raise_for_status()
            return ExecuteResponse.model_validate(response.json())
        finally:
            await self._close_client(client)

    async def execute_python(self, code: str, timeout: int = 30) -> ExecuteResponse:
        """Legacy compatibility wrapper around shell-command execution."""
        return await self.execute_command(code, timeout=timeout)

    async def simulate(
        self,
        script_path: str = "script.py",
        script_content: str | None = None,
        backend: SimulatorBackendType | None = None,
        smoke_test_mode: bool | None = None,
    ) -> BenchmarkToolResponse:
        """Trigger physics simulation via worker."""
        resolved_backend = backend or get_default_simulator_backend()
        if self.controller_url:
            payload = {
                "script_path": script_path,
                "agent_role": self.agent_role,
                "backend": resolved_backend,
                "smoke_test_mode": smoke_test_mode,
            }
            if script_content is not None:
                raise NotImplementedError(
                    "controller script-tools proxy does not accept script_content"
                )
            parsed = await self._call_controller_script_tool(
                "simulate", payload, stage=self.agent_role
            )
            if parsed is None:
                raise RuntimeError("controller script-tools proxy unavailable")
            await self._sync_handover_artifacts_to_light(parsed)
            return parsed

        client = await self._get_client()
        try:
            payload = {"script_path": script_path, "backend": resolved_backend}
            if script_content is not None:
                payload["script_content"] = script_content
            if smoke_test_mode is not None:
                payload["smoke_test_mode"] = smoke_test_mode

            await self._add_bundle_to_payload(payload)

            response = await client.post(
                f"{self.heavy_url}/benchmark/simulate",
                json=payload,
                headers=self._request_headers(stage=self.agent_role),
                timeout=1000.0,
            )
            response.raise_for_status()
            parsed = BenchmarkToolResponse.model_validate(response.json())
            await self._sync_handover_artifacts_to_light(parsed)
            return parsed
        finally:
            await self._close_client(client)

    async def heavy_ready(self) -> bool:
        """Check whether the heavy worker is ready to accept a new job."""
        client = await self._get_client()
        try:
            response = await client.get(
                f"{self.heavy_url}/ready",
                headers=self.headers,
                timeout=5.0,
            )
            return response.status_code == 200
        finally:
            await self._close_client(client)

    async def validate(
        self, script_path: str = "script.py", script_content: str | None = None
    ) -> BenchmarkToolResponse:
        """Trigger geometric validation via worker."""
        if self.controller_url:
            payload = {
                "script_path": script_path,
                "agent_role": self.agent_role,
            }
            if script_content is not None:
                raise NotImplementedError(
                    "controller script-tools proxy does not accept script_content"
                )
            parsed = await self._call_controller_script_tool(
                "validate", payload, stage=self.agent_role
            )
            if parsed is None:
                raise RuntimeError("controller script-tools proxy unavailable")
            await self._sync_handover_artifacts_to_light(parsed)
            return parsed

        client = await self._get_client()
        try:
            payload = {"script_path": script_path}
            if script_content is not None:
                payload["script_content"] = script_content

            await self._add_bundle_to_payload(payload)

            response = await client.post(
                f"{self.heavy_url}/benchmark/validate",
                json=payload,
                headers=self._request_headers(stage=self.agent_role),
                timeout=60.0,
            )
            response.raise_for_status()
            parsed = BenchmarkToolResponse.model_validate(response.json())
            await self._sync_handover_artifacts_to_light(parsed)
            return parsed
        finally:
            await self._close_client(client)

    async def verify(
        self,
        script_path: str = "script.py",
        script_content: str | None = None,
        *,
        backend: SimulatorBackendType | None = None,
        jitter_range: tuple[float, float, float] | None = None,
        num_scenes: int | None = None,
        duration: float | None = None,
        seed: int | None = None,
        smoke_test_mode: bool | None = None,
    ) -> BenchmarkToolResponse:
        """Trigger runtime-randomization verification via worker."""
        resolved_backend = backend or get_default_simulator_backend()
        payload: dict[str, Any] = {
            "script_path": script_path,
            "agent_role": self.agent_role,
            "backend": resolved_backend,
        }
        if jitter_range is not None:
            payload["jitter_range"] = jitter_range
        if num_scenes is not None:
            payload["num_scenes"] = num_scenes
        if duration is not None:
            payload["duration"] = duration
        if seed is not None:
            payload["seed"] = seed
        if smoke_test_mode is not None:
            payload["smoke_test_mode"] = smoke_test_mode
        else:
            payload["smoke_test_mode"] = _default_smoke_test_mode()
        if script_content is not None:
            raise NotImplementedError(
                "controller script-tools proxy does not accept script_content"
            )

        if self.controller_url:
            parsed = await self._call_controller_script_tool(
                "verify", payload, stage=self.agent_role
            )
            if parsed is None:
                raise RuntimeError("controller script-tools proxy unavailable")
            await self._sync_handover_artifacts_to_light(parsed)
            return parsed

        client = await self._get_client()
        try:
            direct_payload = {
                "script_path": script_path,
                "backend": resolved_backend,
            }
            if jitter_range is not None:
                direct_payload["jitter_range"] = jitter_range
            if num_scenes is not None:
                direct_payload["num_scenes"] = num_scenes
            if duration is not None:
                direct_payload["duration"] = duration
            if seed is not None:
                direct_payload["seed"] = seed
            if smoke_test_mode is not None:
                direct_payload["smoke_test_mode"] = smoke_test_mode
            else:
                direct_payload["smoke_test_mode"] = _default_smoke_test_mode()

            await self._add_bundle_to_payload(direct_payload)

            response = await client.post(
                f"{self.heavy_url}/benchmark/verify",
                json=direct_payload,
                headers=self._request_headers(stage=self.agent_role),
                timeout=1000.0,
            )
            response.raise_for_status()
            parsed = BenchmarkToolResponse.model_validate(response.json())
            await self._sync_handover_artifacts_to_light(parsed)
            return parsed
        finally:
            await self._close_client(client)

    async def analyze(
        self,
        method: ManufacturingMethod,
        script_path: str = "script.py",
        script_content: str | None = None,
        quantity: int = 1,
    ) -> WorkbenchResult:
        """Trigger manufacturing analysis via worker."""
        client = await self._get_client()
        try:
            payload = {
                "script_path": script_path,
                "method": method,
                "quantity": quantity,
            }
            if script_content is not None:
                payload["script_content"] = script_content

            await self._add_bundle_to_payload(payload)

            response = await client.post(
                f"{self.heavy_url}/benchmark/analyze",
                json=payload,
                headers=self._request_headers(stage=self.agent_role),
                timeout=60.0,
            )
            response.raise_for_status()
            return WorkbenchResult.model_validate(response.json())
        finally:
            await self._close_client(client)

    async def submit(
        self,
        script_path: str = "script.py",
        script_content: str | None = None,
        reviewer_stage: ReviewerStage | None = None,
    ) -> BenchmarkToolResponse:
        """Trigger handover to review via worker."""
        effective_stage = reviewer_stage or self._default_reviewer_stage(
            self.agent_role
        )
        if self.controller_url:
            payload = {
                "script_path": script_path,
                "agent_role": self.agent_role,
                "reviewer_stage": effective_stage,
            }
            if script_content is not None:
                raise NotImplementedError(
                    "controller script-tools proxy does not accept script_content"
                )
            parsed = await self._call_controller_script_tool(
                "submit", payload, stage=effective_stage or self.agent_role
            )
            if parsed is None:
                raise RuntimeError("controller script-tools proxy unavailable")
            await self._sync_handover_artifacts_to_light(parsed)
            return parsed

        client = await self._get_client()
        try:
            payload = {"script_path": script_path}
            if script_content is not None:
                payload["script_content"] = script_content
            if effective_stage:
                payload["reviewer_stage"] = effective_stage

            await self._add_bundle_to_payload(payload)

            response = await client.post(
                f"{self.heavy_url}/benchmark/submit",
                json=payload,
                headers=self._request_headers(
                    stage=effective_stage or self.agent_role,
                    reviewer_stage=effective_stage,
                ),
                timeout=60.0,
            )
            response.raise_for_status()
            parsed = BenchmarkToolResponse.model_validate(response.json())
            await self._sync_handover_artifacts_to_light(parsed)
            return parsed
        finally:
            await self._close_client(client)

    async def get_health(self) -> dict[str, str]:
        """Check the health of the worker service."""
        client = await self._get_client()
        try:
            response = await client.get(f"{self.base_url}/health", timeout=5.0)
            response.raise_for_status()
            return response.json()
        finally:
            await self._close_client(client)

    async def git_init(self) -> bool:
        """Initialize a git repository in the workspace."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/git/init",
                headers=self._request_headers(stage=self.agent_role),
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == ResponseStatus.SUCCESS
        finally:
            await self._close_client(client)

    async def git_commit(self, message: str) -> GitCommitResponse:
        """Commit changes and sync to S3."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/git/commit",
                json={"message": message},
                headers=self._request_headers(stage=self.agent_role),
                timeout=30.0,
            )
            response.raise_for_status()
            return GitCommitResponse.model_validate(response.json())
        finally:
            await self._close_client(client)

    async def git_status(self) -> GitStatusResponse:
        """Get repository status."""
        client = await self._get_client()
        try:
            response = await client.get(
                f"{self.base_url}/git/status",
                headers=self._request_headers(stage=self.agent_role),
                timeout=10.0,
            )
            response.raise_for_status()
            return GitStatusResponse.model_validate(response.json())
        finally:
            await self._close_client(client)

    async def git_resolve(
        self, file_path: str, strategy: Literal["ours", "theirs"]
    ) -> bool:
        """Resolve a merge conflict."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/git/resolve",
                json={"file_path": file_path, "strategy": strategy},
                headers=self._request_headers(stage=self.agent_role),
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == ResponseStatus.SUCCESS
        finally:
            await self._close_client(client)

    async def git_merge_abort(self) -> bool:
        """Abort a merge."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/git/merge/abort",
                headers=self.headers,
                timeout=10.0,
            )
            response.raise_for_status()
            return response.json()["status"] == ResponseStatus.SUCCESS
        finally:
            await self._close_client(client)

    async def git_merge_complete(self, message: str | None = None) -> GitCommitResponse:
        """Complete a merge."""
        client = await self._get_client()
        try:
            response = await client.post(
                f"{self.base_url}/git/merge/complete",
                json={"message": message},
                headers=self.headers,
                timeout=30.0,
            )
            response.raise_for_status()
            return GitCommitResponse.model_validate(response.json())
        finally:
            await self._close_client(client)
