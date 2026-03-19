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
    HeavySubmitWorkflow,
    HeavyValidationWorkflow,
)
from controller.workflows.simulation import SimulationWorkflow
from shared.ops.workflows import BackupWorkflow, run_backup_activity

TEMPORAL_URL = os.getenv("TEMPORAL_URL", "temporal:7233")
CONNECT_MAX_ATTEMPTS = int(os.getenv("TEMPORAL_CONNECT_MAX_ATTEMPTS", "120"))
CONNECT_RETRY_SECONDS = float(os.getenv("TEMPORAL_CONNECT_RETRY_SECONDS", "1.0"))


async def main():
    # Connect client with retry to tolerate infra startup races.
    attempt = 0
    client = None
    while client is None:
        try:
            client = await Client.connect(TEMPORAL_URL)
        except Exception as exc:
            attempt += 1
            if attempt >= CONNECT_MAX_ATTEMPTS:
                raise RuntimeError(
                    f"Failed to connect to Temporal at {TEMPORAL_URL} "
                    f"after {CONNECT_MAX_ATTEMPTS} attempts"
                ) from exc
            wait_s = min(CONNECT_RETRY_SECONDS * attempt, 10.0)
            print(
                "Temporal connect retry "
                f"{attempt}/{CONNECT_MAX_ATTEMPTS} failed: {exc}. "
                f"Retrying in {wait_s:.1f}s..."
            )
            await asyncio.sleep(wait_s)

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
            HeavySubmitWorkflow,
            BackupWorkflow,
        ],
        activities=[
            compile_mjcf_activity,
            run_simulation_activity,
            render_video_activity,
            upload_to_s3_activity,
            update_trace_activity,
            execute_script_activity,
            run_backup_activity,
        ],
    )
    print("Temporal Worker started on queue: simulation-task-queue")
    await worker.run()


if __name__ == "__main__":
    asyncio.run(main())
