import logging
import subprocess
import sys
import threading
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from fastapi import FastAPI, HTTPException

# Add project root to path so we can import models if they are mounted
sys.path.append("/app")


# Logger setup
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("container_agent")

try:
    from .models import (
        JobRequest,
        JobResponse,
        JobStatus,
        ToolRequest,
        ToolResponse,
    )
except ImportError:
    from models import (
        JobRequest,
        JobResponse,
        JobStatus,
        ToolRequest,
        ToolResponse,
    )


# --- Job Manager ---


class Job:
    def __init__(self, job_id: str, request: JobRequest):
        self.job_id = job_id
        self.request = request
        self.status = JobStatus.QUEUED
        self.stdout = ""
        self.stderr = ""
        self.return_code: int | None = None
        self.error: str | None = None
        self.process: subprocess.Popen | None = None
        self.start_time: float | None = None
        self._lock = threading.Lock()

    def update(self, **kwargs):
        with self._lock:
            for k, v in kwargs.items():
                setattr(self, k, v)


class JobManager:
    def __init__(self):
        self.jobs: dict[str, Job] = {}
        self.executor = ThreadPoolExecutor(max_workers=5)
        self._lock = threading.Lock()

    def create_job(self, request: JobRequest) -> str:
        job_id = str(uuid.uuid4())
        job = Job(job_id, request)
        with self._lock:
            self.jobs[job_id] = job

        self.executor.submit(self._run_job, job)
        return job_id

    def get_job(self, job_id: str) -> Job | None:
        with self._lock:
            return self.jobs.get(job_id)

    def cancel_job(self, job_id: str):
        job = self.get_job(job_id)
        if job:
            with job._lock:
                if job.status in [JobStatus.QUEUED, JobStatus.RUNNING]:
                    if job.process:
                        try:
                            job.process.terminate()
                        except Exception as e:
                            logger.warning(f"Failed to terminate job {job_id}: {e}")
                    job.status = JobStatus.CANCELLED

    def _run_job(self, job: Job):
        # Check if cancelled while in queue
        with job._lock:
            if job.status == JobStatus.CANCELLED:
                return
            job.status = JobStatus.RUNNING
            job.start_time = time.time()

        try:
            req = job.request
            workdir = Path(req.workdir) if req.workdir else Path("/workspace")
            workdir.mkdir(parents=True, exist_ok=True)

            if req.type == "command":
                if not req.command:
                    raise ValueError("Command is empty")
                # Using shell=True for commands to allow string commands
                cmd_args = req.command
                shell = True
            elif req.type == "script":
                if not req.script_name:
                    raise ValueError("Script name is empty")
                script_path = workdir / req.script_name
                if not script_path.exists():
                    # Try absolute path or relative to workspace
                    pass  # subprocess will fail likely

                cmd_args = [sys.executable, req.script_name, *req.args]
                shell = False
            else:
                raise ValueError(f"Unknown job type: {req.type}")

            logger.info(f"Starting job {job.job_id}: {cmd_args}")

            # Check for cancellation again (in case it was cancelled during setup)
            if job.status == JobStatus.CANCELLED:
                return

            process = subprocess.Popen(
                cmd_args,
                shell=shell,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                cwd=workdir,
            )

            job.update(process=process)

            # Check for race condition where cancel came in during Popen creation
            if job.status == JobStatus.CANCELLED:
                process.terminate()
                process.wait()
                return

            stdout, stderr = process.communicate(timeout=req.timeout)

            # Check if cancelled during execution
            with job._lock:
                if job.status == JobStatus.CANCELLED:
                    job.stdout = stdout
                    job.stderr = stderr
                    job.return_code = process.returncode
                    return

            job.update(
                status=JobStatus.COMPLETED,
                stdout=stdout,
                stderr=stderr,
                return_code=process.returncode,
            )

        except subprocess.TimeoutExpired:
            if job.process:
                job.process.kill()
                # Try to capture whatever output was produced
                stdout, stderr = job.process.communicate()
                job.update(stdout=stdout, stderr=stderr)

            job.update(
                status=JobStatus.FAILED,
                error=f"Job timed out after {job.request.timeout}s",
                return_code=124,
            )
        except Exception as e:
            logger.error(f"Job {job.job_id} failed: {e}")
            job.update(status=JobStatus.FAILED, error=str(e), return_code=1)


# --- API ---

app = FastAPI(title="Problemologist Container Agent")
job_manager = JobManager()


@app.get("/health")
def health_check():
    return {"status": "ok"}


@app.post(
    "/jobs",
    response_model=JobResponse,
    responses={
        400: {"description": "Bad Request - Invalid JSON or parameters"},
    },
)
def submit_job(request: JobRequest):
    job_id = job_manager.create_job(request)
    # Return initial status
    return JobResponse(job_id=job_id, status=JobStatus.QUEUED, stdout="", stderr="")


@app.get(
    "/jobs/{job_id}",
    response_model=JobResponse,
    responses={
        404: {"description": "Job not found"},
    },
)
def get_job_status(job_id: str):
    job = job_manager.get_job(job_id)
    if not job:
        raise HTTPException(status_code=404, detail="Job not found")

    return JobResponse(
        job_id=job.job_id,
        status=job.status,
        stdout=job.stdout,
        stderr=job.stderr,
        return_code=job.return_code,
        error=job.error,
    )


@app.post(
    "/exec",
    response_model=ToolResponse,
    responses={
        400: {"description": "Bad Request - Invalid JSON or parameters"},
    },
)
def execute_tool(request: ToolRequest):
    return ToolResponse(
        output=f"Tool {request.tool_name} not implemented in agent yet."
    )


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
