from datetime import timedelta

from temporalio import workflow

from shared.workers.schema import ExecuteResponse, ScriptExecutionRequest


@workflow.defn
class ScriptExecutionWorkflow:
    @workflow.run
    async def run(self, request: ScriptExecutionRequest) -> ExecuteResponse:
        """Workflow to ensure durable execution of scripts on the worker."""
        return await workflow.execute_activity(
            "execute_script_activity",
            request,
            start_to_close_timeout=timedelta(minutes=10),
        )
