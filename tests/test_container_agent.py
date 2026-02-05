import sys
import time
import pytest
from pathlib import Path

# Add src to path to allow imports
PROJECT_ROOT = Path(__file__).parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.append(str(PROJECT_ROOT))

from src.assets.scripts.models import JobRequest, JobType, JobStatus
from src.assets.scripts.container_agent import JobManager

def test_job_cancellation(tmp_path):
    manager = JobManager()

    # Create a long running job
    req = JobRequest(
        type=JobType.COMMAND,
        command="sleep 3",
        timeout=10,
        workdir=str(tmp_path)
    )

    job_id = manager.create_job(req)

    # Wait a moment for it to start
    time.sleep(1)

    job = manager.get_job(job_id)
    assert job.status == JobStatus.RUNNING

    # Cancel the job
    manager.cancel_job(job_id)

    # Give it a moment to update status
    time.sleep(0.5)

    job = manager.get_job(job_id)
    assert job.status == JobStatus.CANCELLED

    # Wait for sleep 3 to finish naturally (if not killed)
    time.sleep(3)

    job = manager.get_job(job_id)
    # It should still be CANCELLED, not overwritten to COMPLETED
    assert job.status == JobStatus.CANCELLED
