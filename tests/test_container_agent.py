import sys
import unittest
from unittest.mock import MagicMock, patch
import threading
import time
from pathlib import Path
import tempfile
import shutil

# Mock FastAPI before importing container_agent
sys.modules["fastapi"] = MagicMock()
sys.modules["uvicorn"] = MagicMock()

# Import the module under test
project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.append(str(project_root))

from src.assets.scripts.container_agent import JobManager, JobStatus
from src.assets.scripts.models import JobRequest, JobType

class TestContainerAgentLogic(unittest.TestCase):
    def setUp(self):
        self.job_manager = JobManager()
        self.test_dir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.test_dir)

    @patch("subprocess.Popen")
    def test_run_job_success(self, mock_popen):
        # Setup mock process
        process_mock = MagicMock()
        process_mock.communicate.return_value = ("stdout", "stderr")
        process_mock.returncode = 0
        mock_popen.return_value = process_mock

        # Use temp dir as workdir
        req = JobRequest(type=JobType.COMMAND, command="echo hello", timeout=1, workdir=self.test_dir)
        job_id = self.job_manager.create_job(req)

        # Wait for job to complete
        start = time.time()
        while time.time() - start < 2:
            job = self.job_manager.get_job(job_id)
            if job.status in [JobStatus.COMPLETED, JobStatus.FAILED]:
                break
            time.sleep(0.1)

        job = self.job_manager.get_job(job_id)
        if job.status == JobStatus.FAILED:
             print(f"Job failed with error: {job.error}")

        self.assertEqual(job.status, JobStatus.COMPLETED)
        self.assertEqual(job.stdout, "stdout")
        self.assertEqual(job.stderr, "stderr")
        self.assertEqual(job.return_code, 0)

    @patch("subprocess.Popen")
    def test_cancel_job(self, mock_popen):
        # Setup mock process that blocks on communicate
        process_mock = MagicMock()

        ready_event = threading.Event()

        def communicate_side_effect(timeout=None):
            ready_event.set()
            time.sleep(0.5)
            return ("partial", "")

        process_mock.communicate.side_effect = communicate_side_effect
        mock_popen.return_value = process_mock

        req = JobRequest(type=JobType.COMMAND, command="sleep 10", timeout=5, workdir=self.test_dir)
        job_id = self.job_manager.create_job(req)

        # Wait until job starts
        if not ready_event.wait(timeout=2):
            # Check if job failed
            job = self.job_manager.get_job(job_id)
            if job.status == JobStatus.FAILED:
                self.fail(f"Job failed with error: {job.error}")
            self.fail("Job did not start in time")

        # Job is running now
        job = self.job_manager.get_job(job_id)
        self.assertEqual(job.status, JobStatus.RUNNING)

        # Cancel job
        self.job_manager.cancel_job(job_id)

        # Verify terminate was called
        process_mock.terminate.assert_called()

        # Wait for thread to finish
        time.sleep(1)

        # Verify final status
        self.assertEqual(job.status, JobStatus.CANCELLED)
