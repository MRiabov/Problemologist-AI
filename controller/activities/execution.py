import os

from temporalio import activity

from controller.clients.worker import WorkerClient
from controller.config.settings import settings
from shared.type_checking import type_check

WORKER_LIGHT_URL = settings.worker_light_url


@type_check
@activity.defn
async def execute_script_activity(data: dict) -> dict:
    """Activity that calls the worker to execute a script."""
    code = data["code"]
    session_id = data["session_id"]
    timeout = data.get("timeout", 30)

    client = WorkerClient(
        base_url=WORKER_LIGHT_URL,
        session_id=session_id,
        heavy_url=settings.worker_heavy_url,
    )
    result = await client.execute_python(code, timeout=timeout)
    return result.model_dump()
