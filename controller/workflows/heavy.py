from datetime import timedelta

from temporalio import workflow

from shared.type_checking import type_check


@type_check
@workflow.defn
class HeavySimulationWorkflow:
    @workflow.run
    async def run(self, params: dict) -> dict:
        """Run physics simulation on a heavy worker."""
        return await workflow.execute_activity(
            "worker_run_simulation",
            params,
            start_to_close_timeout=timedelta(minutes=10),
            # This ensures the activity runs on the heavy worker queue
            task_queue="heavy-tasks-queue",
        )


@type_check
@workflow.defn
class HeavyValidationWorkflow:
    @workflow.run
    async def run(self, params: dict) -> dict:
        """Run geometric validation on a heavy worker."""
        return await workflow.execute_activity(
            "worker_validate_design",
            params,
            start_to_close_timeout=timedelta(minutes=5),
            task_queue="heavy-tasks-queue",
        )


@type_check
@workflow.defn
class HeavyPreviewWorkflow:
    @workflow.run
    async def run(self, params: dict) -> dict:
        """Run design rendering on a heavy worker."""
        return await workflow.execute_activity(
            "worker_preview_design",
            params,
            start_to_close_timeout=timedelta(minutes=2),
            task_queue="heavy-tasks-queue",
        )


@type_check
@workflow.defn
class HeavyAnalyzeWorkflow:
    @workflow.run
    async def run(self, params: dict) -> dict:
        """Run manufacturing analysis on a heavy worker."""
        return await workflow.execute_activity(
            "worker_analyze_design",
            params,
            start_to_close_timeout=timedelta(minutes=5),
            task_queue="heavy-tasks-queue",
        )


@type_check
@workflow.defn
class HeavyCircuitValidationWorkflow:
    @workflow.run
    async def run(self, params: dict) -> dict:
        """Run circuit validation on a heavy worker."""
        return await workflow.execute_activity(
            "worker_validate_circuit",
            params,
            start_to_close_timeout=timedelta(minutes=5),
            task_queue="heavy-tasks-queue",
        )
