import asyncio
import os

import structlog
from temporalio.client import Client
from temporalio.worker import Worker

from shared.logging import configure_logging
from worker_heavy.activities.heavy_tasks import (
    preview_design_activity,
    run_simulation_activity,
    submit_for_review_activity,
    validate_design_activity,
)

configure_logging("worker-heavy-temporal")
logger = structlog.get_logger(__name__)

TEMPORAL_URL = os.getenv("TEMPORAL_URL", "temporal:7233")
CONNECT_MAX_ATTEMPTS = int(os.getenv("TEMPORAL_CONNECT_MAX_ATTEMPTS", "120"))
CONNECT_RETRY_SECONDS = float(os.getenv("TEMPORAL_CONNECT_RETRY_SECONDS", "1.0"))


async def main() -> None:
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
            logger.warning(
                "temporal_connect_retry",
                attempt=attempt,
                max_attempts=CONNECT_MAX_ATTEMPTS,
                error=str(exc),
                retry_in_seconds=wait_s,
            )
            await asyncio.sleep(wait_s)

    worker = Worker(
        client,
        task_queue="heavy-tasks-queue",
        activities=[
            run_simulation_activity,
            validate_design_activity,
            preview_design_activity,
            submit_for_review_activity,
        ],
        max_concurrent_activities=1,
    )
    logger.info("temporal_worker_started", queue="heavy-tasks-queue")
    await worker.run()


if __name__ == "__main__":
    asyncio.run(main())
