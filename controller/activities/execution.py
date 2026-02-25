from temporalio import activity

from controller.clients.worker import WorkerClient
from controller.config.settings import settings
from shared.workers.schema import ExecuteResponse, ScriptExecutionRequest

WORKER_LIGHT_URL = settings.worker_light_url


@activity.defn
async def execute_script_activity(request: ScriptExecutionRequest) -> ExecuteResponse:
    """Activity that calls the worker to execute a script."""
    code = request.code
    session_id = request.session_id
    timeout = request.timeout

    client = WorkerClient(
        base_url=WORKER_LIGHT_URL,
        session_id=session_id,
        heavy_url=settings.worker_heavy_url,
    )
    return await client.execute_python(code, timeout=timeout)
