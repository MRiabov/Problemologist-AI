from datetime import timedelta
from temporalio import activity, workflow
from controller.clients.worker import WorkerClient
import os
from shared.type_checking import type_check

WORKER_URL = os.getenv("WORKER_URL", "http://worker:8001")


@type_check
@activity.defn
async def execute_script_activity(data: dict) -> dict:
    """Activity that calls the worker to execute a script."""
    code = data["code"]
    session_id = data["session_id"]
    timeout = data.get("timeout", 30)

    client = WorkerClient(base_url=WORKER_URL, session_id=session_id)
    result = await client.execute_python(code, timeout=timeout)
    return result.model_dump()


@type_check
@workflow.defn
class ScriptExecutionWorkflow:
    @workflow.run
    async def run(self, data: dict) -> dict:
        """Workflow to ensure durable execution of scripts on the worker."""
        return await workflow.execute_activity(
            execute_script_activity,
            data,
            start_to_close_timeout=timedelta(minutes=10),
        )
