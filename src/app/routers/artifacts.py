from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse
from src.dashboard.utils import resolve_artifact_path
import os

router = APIRouter(prefix="/artifacts", tags=["artifacts"])

@router.get("/{path:path}")
def get_artifact(path: str):
    try:
        file_path = resolve_artifact_path(path)
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

    if not file_path.exists():
         raise HTTPException(status_code=404, detail="Artifact not found")

    return FileResponse(file_path)
