import asyncio
from typing import Any, Optional

from src.environment import tools as env_tools

# Global reference to the active environment (if any)
_ACTIVE_ENV: Any | None = None


def set_active_env(env: Any):
    """Sets the active environment instance for tool interaction."""
    global _ACTIVE_ENV
    _ACTIVE_ENV = env


async def write_script_async(content: str, path: str) -> str:
    """Async wrapper for writing a script."""
    return await asyncio.to_thread(env_tools.write_script, content, path)


async def edit_script_async(path: str, find: str, replace: str) -> str:
    """Async wrapper for editing a script."""
    return await asyncio.to_thread(env_tools.edit_script, path, find, replace)


async def preview_design_async(path: str) -> str:
    """Async wrapper for previewing a design."""
    return await asyncio.to_thread(env_tools.preview_design, path)


async def search_docs_async(query: str) -> str:
    """Async wrapper for searching documentation."""
    return await asyncio.to_thread(env_tools.search_docs, query)


async def read_skill_async(
    skill_name: str, filename: str, resource_type: Optional[str] = None
) -> str:
    """Async wrapper for reading a skill."""
    return await asyncio.to_thread(
        env_tools.read_skill, skill_name, filename, resource_type
    )


async def list_skills_async() -> str:
    """Async wrapper for listing skills."""
    return await asyncio.to_thread(env_tools.list_skills)


async def list_skill_files_async(skill_name: str) -> str:
    """Async wrapper for listing skill files."""
    return await asyncio.to_thread(env_tools.list_skill_files, skill_name)


async def update_skill_async(
    skill_name: str,
    content: str,
    filename: str,
    resource_type: Optional[str] = None,
) -> str:
    """Async wrapper for updating a skill."""
    return await asyncio.to_thread(
        env_tools.update_skill, skill_name, content, filename, resource_type
    )


async def init_skill_async(skill_name: str) -> str:
    """Async wrapper for initializing a skill."""
    return await asyncio.to_thread(env_tools.init_skill, skill_name)


async def package_skill_async(skill_name: str) -> str:
    """Async wrapper for packaging a skill."""
    return await asyncio.to_thread(env_tools.package_skill, skill_name)


async def search_parts_async(query: str) -> str:
    """Async wrapper for searching COTS parts."""
    return await asyncio.to_thread(env_tools.search_parts, query)


async def preview_part_async(part_id: str) -> str:
    """Async wrapper for previewing a COTS part."""
    return await asyncio.to_thread(env_tools.preview_part, part_id)


async def submit_design_async(control_path: str) -> str:
    """Async wrapper for submitting a design."""
    global _ACTIVE_ENV
    if _ACTIVE_ENV:
        # If we have an active CADEnv, use its internal submission logic
        # Run in thread since _submit_design is synchronous and involves MuJoCo
        reward, report, terminated = await asyncio.to_thread(_ACTIVE_ENV._submit_design)
        return (
            f"Submission Result: {report}\nReward: {reward}\nTerminated: {terminated}"
        )
    # Fallback/Mock if no environment is active
    await asyncio.sleep(1)
    return (
        "Warning: No active environment found. Submission simulated. Result: Success."
    )


async def check_manufacturability_async(
    design_file: str, process: str, quantity: int
) -> dict:
    """Async wrapper for check_manufacturability."""
    return await asyncio.to_thread(
        env_tools.check_manufacturability, design_file, process, quantity
    )
