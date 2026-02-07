import os
from fastapi import APIRouter
from typing import List
from pydantic import BaseModel, StrictStr

# In a real scenario, we might fetch this from the Worker via FS or DB.
# For now, we will scan the local 'docs/skills' directory if it exists,
# or return a predefined list if we are in a container without that mount.

router = APIRouter(prefix="/skills", tags=["skills"])

class Skill(BaseModel):
    name: StrictStr
    description: StrictStr

@router.get("/", response_model=List[Skill])
async def list_skills():
    """List available skills."""
    skills_dir = "docs/skills"
    found_skills = []

    if os.path.exists(skills_dir):
        for filename in os.listdir(skills_dir):
            if filename.endswith(".md") or filename.endswith(".py"):
                name = os.path.splitext(filename)[0]
                found_skills.append(Skill(name=name, description="Custom skill"))

    # Fallback/Default skills if none found (or strictly from DB in future)
    if not found_skills:
        return [
            Skill(name="lattice_generator_v2", description="Generates lattice structures"),
            Skill(name="compliant_hinge_pt", description="Parametric compliant hinge"),
            Skill(name="multi_shell_union", description="Robust boolean union for shells"),
        ]

    return found_skills
