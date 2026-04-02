import asyncio
import base64
import hashlib
import time
from contextlib import asynccontextmanager, contextmanager
from pathlib import Path

import httpx
from fastapi import APIRouter, Header, Request
from pydantic import AliasChoices, BaseModel, Field, field_validator, model_validator

from controller.clients.worker import WorkerClient
from controller.config.settings import settings
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.utils import EpisodeIdentity
from shared.enums import AgentName
from shared.logging import bind_log_context
from shared.script_contracts import authored_script_path_for_agent
from shared.simulation.schemas import (
    SimulatorBackendType,
    get_default_simulator_backend,
)
from shared.simulation.smoke_mode import (
    ensure_smoke_test_mode_allowed,
    resolve_default_smoke_test_mode,
)
from shared.workers.schema import (
    BenchmarkToolResponse,
    HeavyValidationResponse,
    PreviewDesignResponse,
    PreviewRenderingType,
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
    bundle_base64: str | None = None
    orbit_pitch: float | list[float] = Field(
        default=-45.0,
        validation_alias=AliasChoices("pitch", "orbit_pitch"),
    )
    orbit_yaw: float | list[float] = Field(
        default=45.0,
        validation_alias=AliasChoices("yaw", "orbit_yaw"),
    )
    rgb: bool | None = None
    depth: bool | None = None
    segmentation: bool | None = None
    drafting: bool = False
    rendering_type: PreviewRenderingType | None = None
    reviewer_stage: ReviewerStage | None = None
    jitter_range: tuple[float, float, float] | None = None
    num_scenes: int | None = None
    duration: float | None = None
    seed: int | None = None
    episode_id: str | None = None

    @field_validator("smoke_test_mode", mode="after")
    @classmethod
    def validate_smoke_test_mode(cls, value: bool | None) -> bool | None:
        return ensure_smoke_test_mode_allowed(value)

    @model_validator(mode="after")
    def normalize_script_path(self) -> "ScriptToolRequest":
        if self.script_path == "script.py":
            self.script_path = authored_script_path_for_agent(self.agent_role)
        return self


@asynccontextmanager
async def _controller_script_middleware(
    session_id: str, agent_role: AgentName, request: Request, episode_id: str | None
):
    identity = EpisodeIdentity.from_context(
        session_id=session_id, episode_id=episode_id
    )
    async with httpx.AsyncClient() as http_client:
        client = WorkerClient(
            base_url=settings.worker_light_url,
            session_id=session_id,
            http_client=http_client,
            heavy_url=settings.worker_heavy_url,
            controller_url="",
            agent_role=agent_role,
            light_transport=settings.worker_light_transport,
        )
        middleware = RemoteFilesystemMiddleware(
            client,
            temporal_client=getattr(request.app.state, "temporal_client", None),
            agent_role=agent_role,
            episode_id=str(identity.episode_id),
        )
        yield middleware


@contextmanager
def _script_log_context(payload: ScriptToolRequest, session_id: str):
    identity = EpisodeIdentity.from_context(
        session_id=session_id, episode_id=payload.episode_id
    )
    stage = payload.reviewer_stage or payload.agent_role.value
    with bind_log_context(
        session_id=identity.session_id,
        episode_id=str(identity.episode_id),
        agent_role=payload.agent_role.value,
        stage=stage,
    ):
        yield


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


async def _collect_render_blobs(
    middleware: RemoteFilesystemMiddleware, render_paths: list[str]
) -> dict[str, str]:
    render_blobs_base64: dict[str, str] = {}
    render_image_paths: list[str] = []
    manifest_candidates: list[str] = []

    for raw_path in render_paths:
        rel_path = str(Path(raw_path))
        suffix = Path(rel_path).suffix.lower()
        if suffix not in {".png", ".jpg", ".jpeg", ".mp4"}:
            continue

        manifest_path = str(Path(rel_path).parent / "render_manifest.json")
        if manifest_path not in manifest_candidates:
            manifest_candidates.append(manifest_path)

        try:
            if not await middleware.client.exists(rel_path):
                continue
            render_blobs_base64[rel_path] = base64.b64encode(
                await middleware.client.read_file_binary(rel_path)
            ).decode("ascii")
            if suffix in {".png", ".jpg", ".jpeg"}:
                render_image_paths.append(rel_path)
        except Exception:
            continue

    manifest_found = False
    for manifest_path in manifest_candidates:
        if not await middleware.client.exists(manifest_path):
            continue
        render_blobs_base64[manifest_path] = base64.b64encode(
            await middleware.client.read_file_binary(manifest_path)
        ).decode("ascii")
        manifest_found = True

    if render_image_paths and not manifest_found:
        raise ValueError(
            "bundle-local render_manifest.json missing for latest preview bundle"
        )

    if render_image_paths and not any(
        path.endswith("render_manifest.json") for path in render_blobs_base64
    ):
        raise ValueError(
            "bundle-local render_manifest.json missing for latest preview bundle"
        )

    return render_blobs_base64


@router.post("/validate", response_model=BenchmarkToolResponse)
async def validate_script(
    request: Request,
    payload: ScriptToolRequest,
    x_session_id: str = Header(...),
):
    with _script_log_context(payload, x_session_id):
        async with _controller_script_middleware(
            x_session_id, payload.agent_role, request, payload.episode_id
        ) as middleware:
            result = await middleware.validate(payload.script_path)
            if isinstance(result, BenchmarkToolResponse):
                await middleware.client._sync_handover_artifacts_to_light(result)
                return result
            if isinstance(result, HeavyValidationResponse) and result.artifacts:
                response = BenchmarkToolResponse(
                    success=result.success,
                    message=result.message or "Validation completed",
                    confidence="high",
                    artifacts=result.artifacts,
                )
                await middleware.client._sync_handover_artifacts_to_light(response)
                return response
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
    with _script_log_context(payload, x_session_id):
        ensure_smoke_test_mode_allowed(
            payload.smoke_test_mode,
            integration_enabled=settings.is_integration_test,
        )
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
            if response.artifacts:
                response.artifacts.render_blobs_base64 = await _collect_render_blobs(
                    middleware,
                    response.artifacts.render_paths,
                )
            await middleware.client._sync_handover_artifacts_to_light(response)
            return response


@router.post("/verify", response_model=BenchmarkToolResponse)
async def verify_script(
    request: Request,
    payload: ScriptToolRequest,
    x_session_id: str = Header(...),
):
    with _script_log_context(payload, x_session_id):
        smoke_test_mode = payload.smoke_test_mode
        if smoke_test_mode is None:
            smoke_test_mode = resolve_default_smoke_test_mode(
                integration_enabled=settings.is_integration_test
            )
        else:
            ensure_smoke_test_mode_allowed(
                smoke_test_mode,
                integration_enabled=settings.is_integration_test,
            )
        async with _controller_script_middleware(
            x_session_id, payload.agent_role, request, payload.episode_id
        ) as middleware:
            result = await _retry_busy(
                lambda: middleware.verify(
                    payload.script_path,
                    backend=payload.backend,
                    smoke_test_mode=smoke_test_mode,
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


@router.post("/preview", response_model=PreviewDesignResponse)
async def preview_script(
    request: Request,
    payload: ScriptToolRequest,
    x_session_id: str = Header(...),
):
    with _script_log_context(payload, x_session_id):
        async with _controller_script_middleware(
            x_session_id, payload.agent_role, request, payload.episode_id
        ) as middleware:
            result = await middleware.preview(
                payload.script_path,
                orbit_pitch=payload.orbit_pitch,
                orbit_yaw=payload.orbit_yaw,
                rgb=payload.rgb,
                depth=payload.depth,
                segmentation=payload.segmentation,
                drafting=payload.drafting,
                rendering_type=payload.rendering_type,
                bundle_base64=payload.bundle_base64,
                smoke_test_mode=payload.smoke_test_mode,
            )
            return result


@router.post("/submit", response_model=BenchmarkToolResponse)
async def submit_script(
    request: Request,
    payload: ScriptToolRequest,
    x_session_id: str = Header(...),
):
    with _script_log_context(payload, x_session_id):
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
