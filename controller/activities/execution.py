import structlog
from temporalio import activity

from controller.clients.worker import WorkerClient
from controller.config.settings import settings
from shared.workers.schema import ExecuteResponse, ScriptExecutionRequest

WORKER_LIGHT_URL = settings.worker_light_url
_INT186_FORCE_INFRA_MARKER = "INT186_FORCE_INFRA_UNAVAILABLE"
_SYSTEM_RETRY_EXHAUSTED_MARKER = "SYSTEM_TOOL_RETRY_EXHAUSTED"
logger = structlog.get_logger(__name__)


@activity.defn
async def execute_script_activity(request: ScriptExecutionRequest) -> ExecuteResponse:
    """Activity that calls the worker to execute a shell command."""
    command = request.code
    session_id = request.session_id
    timeout = request.timeout
    attempt = activity.info().attempt

    if _INT186_FORCE_INFRA_MARKER in command:
        logger.error(
            "execute_script_activity_attempt",
            session_id=session_id,
            attempt=attempt,
            mode="forced_infra_unavailable",
        )
        raise RuntimeError(
            f"{_SYSTEM_RETRY_EXHAUSTED_MARKER}: forced infra unavailability"
        )

    client = WorkerClient(
        base_url=WORKER_LIGHT_URL,
        session_id=session_id,
        heavy_url=settings.worker_heavy_url,
    )
    return await client.execute_command(
        command, timeout=timeout, episode_id=request.episode_id
    )
