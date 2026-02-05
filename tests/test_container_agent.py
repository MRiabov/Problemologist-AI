
import sys
import threading
import time
import subprocess
from unittest.mock import MagicMock, patch
import pytest

from src.assets.scripts.container_agent import JobManager, JobRequest, JobStatus

class TestContainerAgentLogic:
    def test_job_execution_success(self):
        """
        Verifies that a job runs successfully using subprocess.Popen.
        """
        manager = JobManager()

        mock_process = MagicMock()
        mock_process.communicate.return_value = ("stdout output", "stderr output")
        mock_process.returncode = 0

        with patch("subprocess.Popen", return_value=mock_process) as mock_popen, \
             patch("pathlib.Path.mkdir"):

            job_id = manager.create_job(JobRequest(type="command", command="echo hello", workdir="/tmp"))

            # Wait for execution to complete
            start = time.time()
            job = manager.get_job(job_id)
            while job.status in [JobStatus.QUEUED, JobStatus.RUNNING] and time.time() - start < 2:
                time.sleep(0.05)
                job = manager.get_job(job_id)

            assert job.status == JobStatus.COMPLETED
            assert job.stdout == "stdout output"
            assert job.stderr == "stderr output"
            assert job.return_code == 0

            mock_popen.assert_called_once()
            # Verify arguments passed to Popen
            args, kwargs = mock_popen.call_args
            assert args[0] == "echo hello"
            assert kwargs["shell"] is True
            assert kwargs["stdout"] == subprocess.PIPE
            assert kwargs["stderr"] == subprocess.PIPE

    def test_job_cancellation_kills_process(self):
        """
        This test verifies that cancel_job kills the process.
        """
        manager = JobManager()

        # We need to mock Popen to simulate a running process that gets terminated
        mock_process = MagicMock()
        mock_process.poll.return_value = None # Initially running

        # communicate should block until terminated or timeout
        def communicate_side_effect(*args, **kwargs):
            # Simulate blocking wait until terminated
            # We check if terminate was called by checking if poll returns something or if we set a flag
            start = time.time()
            while time.time() - start < 2:
                if mock_process.poll.return_value is not None:
                    # Terminated
                    return "partial stdout", "partial stderr"
                time.sleep(0.05)
            # If we reach here, it means it wasn't terminated in time or just finished normally
            return "timeout", "error"

        mock_process.communicate.side_effect = communicate_side_effect
        mock_process.returncode = None

        # If terminate is called, we update poll return value so communicate loop exits
        def terminate_side_effect():
            mock_process.poll.return_value = -15
            mock_process.returncode = -15

        mock_process.terminate.side_effect = terminate_side_effect

        with patch("subprocess.Popen", return_value=mock_process) as mock_popen, \
             patch("pathlib.Path.mkdir"):

            job_id = manager.create_job(JobRequest(type="command", command="sleep 10", workdir="/tmp"))

            time.sleep(0.1) # Wait for thread to start Popen

            job = manager.get_job(job_id)
            assert job.status == JobStatus.RUNNING
            assert job.process is mock_process

            # Cancel the job
            manager.cancel_job(job_id)

            # Verify terminate was called
            mock_process.terminate.assert_called_once()

            # Wait for thread to finish up
            time.sleep(0.1)

            job = manager.get_job(job_id)
            assert job.status == JobStatus.CANCELLED
