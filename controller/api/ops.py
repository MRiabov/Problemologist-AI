import os

import structlog
from fastapi import APIRouter, Depends, Header, HTTPException, Request
from pydantic import BaseModel

from shared.models.schemas import BackupParams
from shared.ops.workflows import BackupWorkflow

logger = structlog.get_logger(__name__)

router = APIRouter(prefix="/ops", tags=["Operations"])

BACKUP_SECRET = os.getenv("BACKUP_SECRET", "change-me-in-production")


async def verify_backup_secret(x_backup_secret: str = Header(None)):
    """
    Simple header-based secret verification.
    """
    if not x_backup_secret or x_backup_secret != BACKUP_SECRET:
        logger.warning("Unauthorized backup attempt")
        raise HTTPException(status_code=403, detail="Invalid backup secret")


class BackupTriggerResponse(BaseModel):
    workflow_id: str
    status: str
    detail: str


@router.post("/backup", status_code=202, response_model=BackupTriggerResponse)
async def trigger_backup(request: Request, _=Depends(verify_backup_secret)):
    """
    Trigger the automated backup workflow.
    """
    # Assuming temporal_client is attached to app state during startup
    try:
        client = request.app.state.temporal_client
    except AttributeError:
        logger.error("Temporal client not found in app state")
        raise HTTPException(status_code=500, detail="Temporal client not initialized")

    params = BackupParams(
        db_url=os.getenv("DATABASE_URL"),
        s3_bucket=os.getenv("BACKUP_S3_BUCKET"),
        source_bucket=os.getenv("ASSET_S3_BUCKET"),
        backup_bucket=os.getenv("BACKUP_S3_BUCKET"),
    )

    # Validate that we have at least some parameters to work with
    if not any(v is not None for v in params.model_dump().values()):
        logger.error("Missing backup configuration", params=params)
        raise HTTPException(status_code=500, detail="Backup configuration missing")

    workflow_id = f"backup-{os.urandom(4).hex()}"

    logger.info("Starting backup workflow", workflow_id=workflow_id)

    handle = await client.start_workflow(
        BackupWorkflow.run,
        params,
        id=workflow_id,
        task_queue="ops-tasks",
    )

    return BackupTriggerResponse(
        workflow_id=handle.id,
        status="Accepted",
        detail="Backup workflow initiated",
    )
