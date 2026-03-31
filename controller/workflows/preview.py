from datetime import timedelta

from temporalio import workflow
from temporalio.common import RetryPolicy

from shared.workers.schema import PreviewDesignResponse, PreviewWorkflowParams


@workflow.defn
class PreviewWorkflow:
    @workflow.run
    async def run(self, params: PreviewWorkflowParams) -> PreviewDesignResponse:
        """Run a single preview render through controller orchestration."""
        return await workflow.execute_activity(
            "controller_preview_render_activity",
            params,
            start_to_close_timeout=timedelta(minutes=5),
            retry_policy=RetryPolicy(maximum_attempts=3),
            task_queue="simulation-task-queue",
        )
