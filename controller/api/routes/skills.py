from pathlib import Path

from fastapi import APIRouter
from pydantic import BaseModel, StrictStr

# Expose only the checked-in repo skill tree. Hidden overlay skill stores such
# as `.agent/skills/` are intentionally excluded from this endpoint.

router = APIRouter(prefix="/skills", tags=["skills"])


class Skill(BaseModel):
    name: StrictStr
    description: StrictStr


@router.get("/", response_model=list[Skill])
async def list_skills():
    """List available skills."""
    skills_dir = Path("skills")
    found_skills = []

    if skills_dir.exists():
        for item in sorted(skills_dir.iterdir(), key=lambda p: p.name):
            if item.is_dir():
                found_skills.append(
                    Skill(name=item.name, description=f"Skill from {item.name}")
                )

    return found_skills
