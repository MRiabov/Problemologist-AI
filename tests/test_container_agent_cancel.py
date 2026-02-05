import unittest
from unittest.mock import MagicMock, patch
import time
import tempfile
import shutil
from src.assets.scripts.container_agent import JobManager, JobRequest, JobStatus

class TestContainerAgentCancel(unittest.TestCase):
    def setUp(self):
        self.job_manager = JobManager()
        self.temp_dir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.temp_dir)

    @patch("subprocess.Popen")
    def test_cancel_running_job(self, mock_popen):
        # Setup mock process
        mock_process = MagicMock()
        mock_process.poll.return_value = None
        mock_process.returncode = None

        # When communicate is called, we simulate a delay to allow cancellation
        def delayed_communicate(timeout=None):
            time.sleep(0.2) # Simulate work
            return "out", "err"

        mock_process.communicate.side_effect = delayed_communicate
        mock_popen.return_value = mock_process

        # Create request
        req = JobRequest(type="command", command="sleep 10", timeout=5, workdir=self.temp_dir)
        job_id = self.job_manager.create_job(req)

        # Wait for job to start
        start = time.time()
        job = None
        while time.time() - start < 1.0:
            job = self.job_manager.get_job(job_id)
            if job and job.status == JobStatus.RUNNING:
                break
            time.sleep(0.01)

        self.assertIsNotNone(job)
        self.assertEqual(job.status, JobStatus.RUNNING)
        self.assertIsNotNone(job.process)

        # Cancel job
        self.job_manager.cancel_job(job_id)

        # Wait for thread to finish
        time.sleep(0.3)

        # Verify status
        self.assertEqual(job.status, JobStatus.CANCELLED)

        # Verify terminate was called
        mock_process.terminate.assert_called()

    @patch("subprocess.Popen")
    def test_cancel_during_startup_race(self, mock_popen):
        # Setup mock process
        mock_process = MagicMock()
        mock_process.poll.return_value = None
        mock_process.communicate.return_value = ("out", "err")
        mock_popen.return_value = mock_process

        # Define side effect to trigger cancellation during Popen
        # We define it before creating job, but we need to access job_manager
        def cancel_during_popen(*args, **kwargs):
            # This runs inside _run_job thread
            # Get job_id safely
            keys = list(self.job_manager.jobs.keys())
            if keys:
                jid = keys[0]
                self.job_manager.cancel_job(jid)
            return mock_process

        mock_popen.side_effect = cancel_during_popen

        # Create request
        req = JobRequest(type="command", command="sleep 10", workdir=self.temp_dir)
        job_id = self.job_manager.create_job(req)

        # Wait for job to process
        # We need to wait enough for executor to pick it up
        time.sleep(0.5)

        job = self.job_manager.get_job(job_id)
        self.assertEqual(job.status, JobStatus.CANCELLED)

        # Verify kill was called (handling the race)
        mock_process.kill.assert_called()

if __name__ == '__main__':
    unittest.main()
