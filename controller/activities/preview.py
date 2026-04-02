import structlog
from temporalio import activity

from controller.clients.worker import WorkerClient
from controller.config.settings import settings
from shared.workers.schema import PreviewDesignResponse, PreviewWorkflowParams

logger = structlog.get_logger(__name__)

WORKER_LIGHT_URL = settings.worker_light_url
WORKER_HEAVY_URL = settings.worker_heavy_url


@activity.defn(name="controller_preview_render_activity")
async def preview_render_activity(
    params: PreviewWorkflowParams,
) -> PreviewDesignResponse:
    """Render a single preview by delegating to worker-light."""
    logger.info(
        "controller_preview_render_activity",
        session_id=params.session_id,
        agent_role=params.agent_role,
        script_path=params.script_path,
        rendering_type=(params.rendering_type.value if params.rendering_type else None),
    )
    client = WorkerClient(
        base_url=WORKER_LIGHT_URL,
        session_id=params.session_id,
        heavy_url=WORKER_HEAVY_URL,
        controller_url="",
        agent_role=params.agent_role,
        light_transport=settings.worker_light_transport,
    )
    return await client.preview(
        script_path=params.script_path,
        script_content=params.script_content,
        orbit_pitch=params.orbit_pitch,
        orbit_yaw=params.orbit_yaw,
        rgb=params.rgb,
        depth=params.depth,
        segmentation=params.segmentation,
        drafting=params.drafting,
        rendering_type=params.rendering_type,
        bundle_base64=params.bundle_base64,
        smoke_test_mode=params.smoke_test_mode,
    )
