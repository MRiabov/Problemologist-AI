from temporalio import activity
from controller.clients.worker import WorkerClient
import os
from shared.type_checking import type_check

# Configuration for activities
WORKER_URL = os.getenv("WORKER_URL", "http://worker:8001")


@type_check
@activity.defn
async def compile_mjcf_activity(compound_json: str) -> str:
    """Mock activity to compile MJCF."""
    # In a real scenario, this would call the worker to generate MJCF
    return "mjcf_data"


@type_check
@activity.defn
async def run_simulation_activity(mjcf_data: str) -> str:
    """Mock activity to run simulation on worker."""
    client = WorkerClient(base_url=WORKER_URL, session_id="simulation")
    # This is a placeholder for the actual simulation execution on the worker
    code = "print('Running simulation...')"
    await client.execute_python(code)
    return "sim_results"


@type_check
@activity.defn
async def render_video_activity(sim_results: str) -> str:
    """Mock activity to render simulation video."""
    return "video_path"


@type_check
@activity.defn
async def upload_to_s3_activity(video_path: str) -> str:
    """Mock activity to upload video to S3."""
    return "https://s3.railway.app/backups/sim_video_latest.mp4"


@type_check
@activity.defn
async def update_trace_activity(s3_url: str) -> bool:
    """Mock activity to update postgres trace."""
    return True