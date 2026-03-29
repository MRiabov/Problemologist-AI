from fastapi import APIRouter
from pydantic import BaseModel, StrictStr

from shared.skills import iter_skill_catalog_entries

router = APIRouter(prefix="/skills", tags=["skills"])


class Skill(BaseModel):
    name: StrictStr
    description: StrictStr


@router.get("/", response_model=list[Skill])
async def list_skills():
    """List available skills."""
    return [
        Skill(name=skill_name, description=description)
        for skill_name, description in iter_skill_catalog_entries()
    ]
