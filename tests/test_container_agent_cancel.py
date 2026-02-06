import pytest
import time
import subprocess
import tempfile
import shutil
from unittest.mock import MagicMock, patch
from src.assets.scripts.container_agent import JobManager, JobRequest, JobStatus, Job

@pytest.fixture
def job_manager():
    return JobManager()

@pytest.fixture
def temp_workdir():
    d = tempfile.mkdtemp()
    yield d
    shutil.rmtree(d)

def test_cancel_job_terminates_process(job_manager, temp_workdir):
    request = JobRequest(type="command", command="sleep 10", timeout=5, workdir=temp_workdir)

    mock_process = MagicMock()
    mock_process.returncode = None

    # communicate simulates blocking
    def side_effect(timeout=None):
        time.sleep(2)
        return "stdout", "stderr"
    mock_process.communicate.side_effect = side_effect

    with patch("subprocess.Popen", return_value=mock_process) as mock_popen:
        job_id = job_manager.create_job(request)

        # Wait for job to start
        max_retries = 10
        for _ in range(max_retries):
            job = job_manager.get_job(job_id)
            if job.status == JobStatus.RUNNING:
                break
            time.sleep(0.1)

        assert job.status == JobStatus.RUNNING

        # Cancel
        job_manager.cancel_job(job_id)

        # Wait for thread to process cancellation if needed
        time.sleep(0.1)

        assert job.status == JobStatus.CANCELLED
        mock_process.terminate.assert_called_once()

def test_cancel_job_before_start_logic(job_manager, temp_workdir):
    # Test _run_job logic directly to ensure it respects pre-existing cancellation
    request = JobRequest(type="command", command="echo hello", timeout=5, workdir=temp_workdir)
    job = Job("test-id-1", request)
    job.status = JobStatus.CANCELLED

    with patch("subprocess.Popen") as mock_popen:
        job_manager._run_job(job)

        # Should return immediately without calling Popen
        mock_popen.assert_not_called()

def test_cancel_job_during_popen_logic(job_manager, temp_workdir):
    # Test race condition where cancel happens during Popen creation
    request = JobRequest(type="command", command="echo hello", timeout=5, workdir=temp_workdir)
    job = Job("test-id-2", request)

    mock_process = MagicMock()

    def popen_side_effect(*args, **kwargs):
        # Simulate cancel happening right now
        job.status = JobStatus.CANCELLED
        return mock_process

    with patch("subprocess.Popen", side_effect=popen_side_effect) as mock_popen:
        job_manager._run_job(job)

        mock_popen.assert_called()
        mock_process.terminate.assert_called()
        # Should return and not call communicate
        mock_process.communicate.assert_not_called()

def test_timeout_kills_process(job_manager, temp_workdir):
    request = JobRequest(type="command", command="sleep 10", timeout=1, workdir=temp_workdir)

    mock_process = MagicMock()

    # First call raises TimeoutExpired, second call returns output (after kill)
    mock_process.communicate.side_effect = [
        subprocess.TimeoutExpired(cmd="sleep 10", timeout=1),
        ("stdout", "stderr")
    ]

    with patch("subprocess.Popen", return_value=mock_process):
        job_id = job_manager.create_job(request)

        # Wait for completion (FAILED)
        max_retries = 20
        for _ in range(max_retries):
            job = job_manager.get_job(job_id)
            if job.status == JobStatus.FAILED:
                break
            time.sleep(0.1)

        assert job.status == JobStatus.FAILED
        assert job.return_code == 124
        mock_process.kill.assert_called_once()
