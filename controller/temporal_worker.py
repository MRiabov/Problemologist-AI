import asyncio
import os

from temporalio.client import Client
from temporalio.worker import Worker

from controller.activities.execution import execute_script_activity
from controller.activities.simulation import (
    compile_mjcf_activity,
    render_video_activity,
    run_simulation_activity,
    update_trace_activity,
    upload_to_s3_activity,
)
from controller.workflows.execution import ScriptExecutionWorkflow
from controller.workflows.heavy import (
    HeavyPreviewWorkflow,
    HeavySimulationWorkflow,
    HeavyValidationWorkflow,
)
from controller.workflows.simulation import SimulationWorkflow

TEMPORAL_URL = os.getenv("TEMPORAL_URL", "temporal:7233")


async def main():
    # Connect client
    client = await Client.connect(TEMPORAL_URL)

    # Run the worker
    worker = Worker(
        client,
        task_queue="simulation-task-queue",
        workflows=[
            SimulationWorkflow,
            ScriptExecutionWorkflow,
            HeavySimulationWorkflow,
            HeavyValidationWorkflow,
            HeavyPreviewWorkflow,
        ],
        activities=[
            compile_mjcf_activity,
            run_simulation_activity,
            render_video_activity,
            upload_to_s3_activity,
            update_trace_activity,
            execute_script_activity,
        ],
    )
    print("Temporal Worker started on queue: simulation-task-queue")
    await worker.run()


if __name__ == "__main__":
    asyncio.run(main())
