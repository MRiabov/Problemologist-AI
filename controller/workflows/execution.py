from datetime import timedelta
from typing import TYPE_CHECKING

from temporalio import workflow

from shared.type_checking import type_check
from shared.workers.schema import ExecuteResponse, ScriptExecutionRequest

if TYPE_CHECKING:
    from controller.activities.execution import execute_script_activity


@type_check
@workflow.defn
class ScriptExecutionWorkflow:
    @workflow.run
    async def run(self, request: ScriptExecutionRequest) -> ExecuteResponse:
        """Workflow to ensure durable execution of scripts on the worker."""
        # We use a string here to avoid importing the activity at runtime,
        # which would pull in restricted libraries like 'httpx' into the workflow sandbox.
        return await workflow.execute_activity(
            "execute_script_activity",
            request,
            start_to_close_timeout=timedelta(minutes=10),
        )
