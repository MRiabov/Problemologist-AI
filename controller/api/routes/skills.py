from pathlib import Path

from fastapi import APIRouter
from pydantic import BaseModel, StrictStr

# In a real scenario, we might fetch this from the Worker via FS or DB.
# For now, we will scan the local 'docs/skills' directory if it exists,
# or return a predefined list if we are in a container without that mount.

router = APIRouter(prefix="/skills", tags=["skills"])


class Skill(BaseModel):
    name: StrictStr
    description: StrictStr


@router.get("/", response_model=list[Skill])
async def list_skills():
    """List available skills."""
    skills_dir = Path(".agent/skills")
    found_skills = []

    if skills_dir.exists():
        for item in skills_dir.iterdir():
            if item.is_dir():
                found_skills.append(
                    Skill(name=item.name, description=f"Skill from {item.name}")
                )

    return found_skills
