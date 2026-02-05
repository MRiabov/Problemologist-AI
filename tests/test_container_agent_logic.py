import sys
import os
import unittest
from unittest.mock import MagicMock, patch
import time
import subprocess
import tempfile
import shutil

# Try to import dependencies, mock if missing
try:
    import fastapi
except ImportError:
    mock_fastapi = MagicMock()
    sys.modules["fastapi"] = mock_fastapi

try:
    import pydantic
except ImportError:
    mock_pydantic = MagicMock()
    sys.modules["pydantic"] = mock_pydantic

    class MockBaseModel:
        def __init__(self, **kwargs):
            for k, v in kwargs.items():
                setattr(self, k, v)
        def __getattr__(self, name):
            return None

    mock_pydantic.BaseModel = MockBaseModel
    mock_pydantic.Field = lambda *args, **kwargs: None

sys.path.append(os.path.abspath("src/assets/scripts"))

from container_agent import JobManager, Job, JobRequest, JobStatus

class TestJobManager(unittest.TestCase):
    def setUp(self):
        self.manager = JobManager()
        self.temp_dir = tempfile.mkdtemp()

    def tearDown(self):
        shutil.rmtree(self.temp_dir)

    @patch("subprocess.Popen")
    def test_run_job_success(self, mock_popen):
        mock_process = MagicMock()
        mock_process.communicate.return_value = ("output", "")
        mock_process.returncode = 0
        mock_popen.return_value = mock_process

        request = JobRequest(type="command", command="echo hello", timeout=1, workdir=self.temp_dir)
        job_id = self.manager.create_job(request)

        self.manager.executor.shutdown(wait=True)

        job = self.manager.get_job(job_id)
        self.assertEqual(job.status, JobStatus.COMPLETED)
        self.assertEqual(job.stdout, "output")
        mock_popen.assert_called()

    @patch("subprocess.Popen")
    def test_run_job_timeout(self, mock_popen):
        mock_process = MagicMock()
        mock_process.communicate.side_effect = [
            subprocess.TimeoutExpired(cmd="cmd", timeout=1),
            ("partial", "error")
        ]
        mock_process.returncode = 124
        mock_popen.return_value = mock_process

        request = JobRequest(type="command", command="sleep 2", timeout=1, workdir=self.temp_dir)
        job_id = self.manager.create_job(request)
        self.manager.executor.shutdown(wait=True)

        job = self.manager.get_job(job_id)
        self.assertEqual(job.status, JobStatus.FAILED)
        mock_process.kill.assert_called()

    @patch("subprocess.Popen")
    def test_cancel_running_job(self, mock_popen):
        mock_process = MagicMock()

        def communicate_side_effect(timeout=None):
            time.sleep(0.5)
            return ("output", "")

        mock_process.communicate.side_effect = communicate_side_effect
        mock_process.returncode = -15
        mock_popen.return_value = mock_process

        request = JobRequest(type="command", command="long run", timeout=5, workdir=self.temp_dir)
        job_id = self.manager.create_job(request)

        time.sleep(0.1)
        self.manager.cancel_job(job_id)

        self.manager.executor.shutdown(wait=True)

        job = self.manager.get_job(job_id)

        self.assertEqual(job.status, JobStatus.CANCELLED)
        mock_process.terminate.assert_called()

    @patch("subprocess.Popen")
    def test_cancel_queued_job(self, mock_popen):
        # We patch _run_job on the instance (method injection) or class
        # Patching on instance is tricky for bound methods passed to executor
        # But we can patch JobManager._run_job

        original_run_job = JobManager._run_job

        def delayed_run_job(self, job):
            time.sleep(0.2)
            original_run_job(self, job)

        with patch.object(JobManager, '_run_job', side_effect=delayed_run_job, autospec=True):
             # We need a new manager to use the patched method?
             # No, if we patch the class, instances use it.
             manager = JobManager()
             request = JobRequest(type="command", command="echo hello", timeout=1, workdir=self.temp_dir)
             job_id = manager.create_job(request)

             # Immediately cancel. It is queued because delayed_run_job is sleeping
             manager.cancel_job(job_id)

             manager.executor.shutdown(wait=True)
             job = manager.get_job(job_id)

             self.assertEqual(job.status, JobStatus.CANCELLED)
             mock_popen.assert_not_called()

if __name__ == "__main__":
    unittest.main()
