import os
import uuid

from temporalio import activity

from controller.clients.worker import WorkerClient
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Asset, Episode
from shared.enums import AssetType, EpisodeStatus
from shared.observability.storage import S3Client, S3Config
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
    """Upload video to S3 using S3Client."""
    config = S3Config(
        endpoint_url=os.getenv("S3_ENDPOINT_URL"),
        access_key_id=os.getenv("AWS_ACCESS_KEY_ID"),
        secret_access_key=os.getenv("AWS_SECRET_ACCESS_KEY"),
        bucket_name=os.getenv("ASSET_S3_BUCKET", "assets"),
        region_name=os.getenv("AWS_REGION", "us-east-1"),
    )
    client = S3Client(config)

    # Use Path for modern path handling
    from pathlib import Path

    object_key = f"videos/{uuid.uuid4()}_{Path(video_path).name}"

    await client.aupload_file(video_path, object_key)
    return object_key


@type_check
@activity.defn
async def update_trace_activity(params: dict) -> bool:
    """Update postgres episode status and add asset record."""
    episode_id = params["episode_id"]
    s3_path = params["s3_path"]
    asset_type = params.get("asset_type", AssetType.VIDEO)

    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = await db.get(Episode, uuid.UUID(episode_id))
        if not episode:
            raise ValueError(f"Episode {episode_id} not found in database")

        episode.status = EpisodeStatus.COMPLETED

        asset = Asset(
            episode_id=uuid.UUID(episode_id),
            asset_type=asset_type,
            s3_path=s3_path,
        )
        db.add(asset)
        await db.commit()

    return True
