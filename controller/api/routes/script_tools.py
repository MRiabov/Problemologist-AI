import asyncio
import hashlib
import time
from contextlib import asynccontextmanager

import httpx
from fastapi import APIRouter, Header, Request
from pydantic import BaseModel, Field

from controller.clients.worker import WorkerClient
from controller.config.settings import settings
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from shared.enums import AgentName
from shared.simulation.schemas import (
    SimulatorBackendType,
    get_default_simulator_backend,
)
from shared.workers.schema import (
    BenchmarkToolResponse,
    ReviewerStage,
    SimulationArtifacts,
    ValidationResultRecord,
)

router = APIRouter(prefix="/script-tools", tags=["script-tools"])


class ScriptToolRequest(BaseModel):
    script_path: str = Field(default="script.py")
    agent_role: AgentName = Field(default=AgentName.ENGINEER_CODER)
    backend: SimulatorBackendType = Field(default_factory=get_default_simulator_backend)
    smoke_test_mode: bool | None = None
    reviewer_stage: ReviewerStage | None = None
    jitter_range: tuple[float, float, float] | None = None
    num_scenes: int | None = None
    duration: float | None = None
    seed: int | None = None
    episode_id: str | None = None


@asynccontextmanager
async def _controller_script_middleware(
    session_id: str, agent_role: AgentName, request: Request, episode_id: str | None
):
    async with httpx.AsyncClient() as http_client:
        client = WorkerClient(
            base_url=settings.worker_light_url,
            session_id=session_id,
            http_client=http_client,
            heavy_url=settings.worker_heavy_url,
            agent_role=agent_role,
        )
        middleware = RemoteFilesystemMiddleware(
            client,
            temporal_client=getattr(request.app.state, "temporal_client", None),
            agent_role=agent_role,
            episode_id=episode_id or session_id,
        )
        yield middleware


async def _retry_busy(callable_):
    delay = 1.5
    max_attempts = 10
    for attempt in range(1, max_attempts + 1):
        try:
            return await callable_()
        except httpx.HTTPStatusError as exc:
            if exc.response.status_code != 503 or attempt == max_attempts:
                raise
            await asyncio.sleep(delay)
            delay = min(delay * 1.5, 5.0)


@router.post("/validate", response_model=BenchmarkToolResponse)
async def validate_script(
    request: Request,
    payload: ScriptToolRequest,
    x_session_id: str = Header(...),
):
    async with _controller_script_middleware(
        x_session_id, payload.agent_role, request, payload.episode_id
    ) as middleware:
        result = await middleware.validate(payload.script_path)
        if isinstance(result, BenchmarkToolResponse):
            await middleware.client._sync_handover_artifacts_to_light(result)
            return result
        if not await middleware.client.exists(payload.script_path):
            raise FileNotFoundError(
                f"script missing while materializing validation record: {payload.script_path}"
            )
        script_content = await middleware.client.read_file(payload.script_path)
        script_sha256 = hashlib.sha256(script_content.encode("utf-8")).hexdigest()

        validation_record = ValidationResultRecord(
            success=result.success,
            message=result.message,
            timestamp=time.time(),
            script_path=payload.script_path,
            script_sha256=script_sha256,
            verification_result=None,
        )
        response = BenchmarkToolResponse(
            success=result.success,
            message=result.message or "Validation completed",
            confidence="high",
            artifacts=SimulationArtifacts(
                validation_results_json=validation_record.model_dump_json(indent=2),
            ),
        )
        await middleware.client._sync_handover_artifacts_to_light(response)
        return response


@router.post("/simulate", response_model=BenchmarkToolResponse)
async def simulate_script(
    request: Request,
    payload: ScriptToolRequest,
    x_session_id: str = Header(...),
):
    async with _controller_script_middleware(
        x_session_id, payload.agent_role, request, payload.episode_id
    ) as middleware:
        result = await middleware.simulate(
            payload.script_path,
            backend=payload.backend,
            smoke_test_mode=payload.smoke_test_mode,
        )
        if isinstance(result, BenchmarkToolResponse):
            await middleware.client._sync_handover_artifacts_to_light(result)
            return result
        response = BenchmarkToolResponse(
            success=result.success,
            message=result.summary,
            confidence=getattr(result, "confidence", "high"),
            artifacts=SimulationArtifacts(
                render_paths=list(getattr(result, "render_paths", [])),
                simulation_result_json=result.model_dump_json(),
                total_cost=getattr(result, "total_cost", None),
                total_weight_g=getattr(result, "total_weight_g", None),
            ),
        )
        await middleware.client._sync_handover_artifacts_to_light(response)
        return response


@router.post("/verify", response_model=BenchmarkToolResponse)
async def verify_script(
    request: Request,
    payload: ScriptToolRequest,
    x_session_id: str = Header(...),
):
    async with _controller_script_middleware(
        x_session_id, payload.agent_role, request, payload.episode_id
    ) as middleware:
        result = await _retry_busy(
            lambda: middleware.verify(
                payload.script_path,
                backend=payload.backend,
                smoke_test_mode=payload.smoke_test_mode,
                jitter_range=payload.jitter_range,
                num_scenes=payload.num_scenes,
                duration=payload.duration,
                seed=payload.seed,
            )
        )
        if isinstance(result, BenchmarkToolResponse):
            await middleware.client._sync_handover_artifacts_to_light(result)
            return result
        response = BenchmarkToolResponse(
            success=result.success,
            message=result.message,
            confidence=getattr(result, "confidence", "high"),
            artifacts=getattr(result, "artifacts", None),
        )
        await middleware.client._sync_handover_artifacts_to_light(response)
        return response


@router.post("/submit", response_model=BenchmarkToolResponse)
async def submit_script(
    request: Request,
    payload: ScriptToolRequest,
    x_session_id: str = Header(...),
):
    async with _controller_script_middleware(
        x_session_id, payload.agent_role, request, payload.episode_id
    ) as middleware:
        result = await _retry_busy(
            lambda: middleware.submit(
                payload.script_path, reviewer_stage=payload.reviewer_stage
            )
        )
        if isinstance(result, BenchmarkToolResponse):
            await middleware.client._sync_handover_artifacts_to_light(result)
            return result
        response = BenchmarkToolResponse(
            success=result.success,
            message=result.message or "Submission completed",
            confidence=getattr(result, "confidence", "high"),
            artifacts=getattr(result, "artifacts", None),
        )
        await middleware.client._sync_handover_artifacts_to_light(response)
        return response
