from datetime import timedelta

from temporalio import workflow

from shared.models.simulation import SimulationResult
from shared.workers.schema import (
    HeavyPreviewParams,
    HeavyPreviewResponse,
    HeavySimulationParams,
    HeavyValidationParams,
    HeavyValidationResponse,
)


@workflow.defn
class HeavySimulationWorkflow:
    @workflow.run
    async def run(self, params: HeavySimulationParams) -> SimulationResult:
        """Run physics simulation on a heavy worker."""
        return await workflow.execute_activity(
            "worker_run_simulation",
            params,
            start_to_close_timeout=timedelta(minutes=10),
            # This ensures the activity runs on the heavy worker queue
            task_queue="heavy-tasks-queue",
        )


@workflow.defn
class HeavyValidationWorkflow:
    @workflow.run
    async def run(self, params: HeavyValidationParams) -> HeavyValidationResponse:
        """Run geometric validation on a heavy worker."""
        return await workflow.execute_activity(
            "worker_validate_design",
            params,
            start_to_close_timeout=timedelta(minutes=5),
            task_queue="heavy-tasks-queue",
        )


@workflow.defn
class HeavyPreviewWorkflow:
    @workflow.run
    async def run(self, params: HeavyPreviewParams) -> HeavyPreviewResponse:
        """Run design rendering on a heavy worker."""
        return await workflow.execute_activity(
            "worker_preview_design",
            params,
            start_to_close_timeout=timedelta(minutes=2),
            task_queue="heavy-tasks-queue",
        )
