from typing import List, Optional
from uuid import UUID
from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from sqlalchemy import select
from sqlalchemy.orm import joinedload

from src.app.dependencies import get_db
from src.app.schemas import EpisodeSummarySchema, EpisodeDetailSchema, StepSchema, ArtifactSchema
from src.environment.persistence import Episode, Step, Artifact

router = APIRouter(prefix="/episodes", tags=["episodes"])

@router.get("/", response_model=List[EpisodeSummarySchema])
def get_episodes(db: Session = Depends(get_db)):
    stmt = select(Episode).order_by(Episode.start_time.desc())
    episodes = db.scalars(stmt).all()
    # Manual mapping if schemas don't match 1:1 perfectly or rely on from_attributes
    return episodes

@router.get("/{episode_id}", response_model=EpisodeDetailSchema)
def get_episode_details(episode_id: str, db: Session = Depends(get_db)):
    try:
        uuid_id = UUID(episode_id)
    except ValueError:
        raise HTTPException(status_code=400, detail="Invalid UUID format")

    stmt = (
        select(Episode)
        .where(Episode.id == uuid_id)
        .options(joinedload(Episode.steps).joinedload(Step.artifacts))
    )
    episode = db.scalars(stmt).unique().one_or_none()

    if not episode:
        raise HTTPException(status_code=404, detail="Episode not found")

    # Pydantic from_attributes should handle the recursion if models match
    # However, StepSchema expects 'artifacts' as List[str] (file paths)
    # but the DB has List[Artifact] objects.
    # We need to transform the data to match the schema.

    steps_data = []
    for step in episode.steps:
        steps_data.append({
            "index": step.sequence_index,
            "type": step.type,
            "agent_role": step.agent_role,
            "content": step.tool_input if step.type == 'tool' else step.content, # Logic mapping
            "tool_name": step.tool_name,
            "tool_input": step.tool_input,
            "tool_output": step.tool_output,
            "metadata": step.metadata_json or {},
            "artifacts": [a.file_path for a in step.artifacts]
        })

    return {
        "id": str(episode.id),
        "name": f"Problem: {episode.problem_id}" if episode.problem_id else "Untitled Episode",
        "steps": steps_data
    }
