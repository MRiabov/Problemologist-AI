import asyncio
from typing import Any, Optional

from src.environment import tools as env_tools

# Global reference to the active environment (if any)
_ACTIVE_ENV: Any | None = None
_CURRENT_ROLE: str | None = None


def set_active_env(env: Any):
    """Sets the active environment instance for tool interaction."""
    global _ACTIVE_ENV
    _ACTIVE_ENV = env


def get_active_env() -> Any | None:
    """Returns the currently active environment instance."""
    return _ACTIVE_ENV


def set_current_role(role: str | None):
    """Sets the current agent role for logging."""
    global _CURRENT_ROLE
    _CURRENT_ROLE = role


async def _run_env_step(tool_idx: int, arguments: str) -> str:
    """Helper to run a step in the active environment or fallback to direct tool call."""
    global _ACTIVE_ENV, _CURRENT_ROLE
    if _ACTIVE_ENV:
        action = {"tool": tool_idx, "arguments": arguments}
        # Run in thread since step might involve MuJoCo or file I/O
        obs, reward, terminated, truncated, info = await asyncio.to_thread(
            _ACTIVE_ENV.step, action, agent_role=_CURRENT_ROLE
        )
        return obs["last_output"]

    # Fallback to direct tool calls if no env is active
    tool_map = {
        0: lambda args: env_tools.write_script(args),
        1: lambda args: env_tools.edit_script("design.py", *args.split("|||", 1))
        if "|||" in args
        else "Error",
        2: lambda args: env_tools.preview_design(args or "design.py"),
        3: lambda args: env_tools.search_docs(args),
        5: lambda args: env_tools.search_parts(args),
        6: lambda args: env_tools.preview_part(args),
        7: lambda args: str(
            env_tools.check_manufacturability("design.py", *args.split("|||", 1))
        )
        if "|||" in args
        else str(env_tools.check_manufacturability("design.py", args or "cnc")),
    }

    func = tool_map.get(tool_idx)
    if func:
        return await asyncio.to_thread(func, arguments)
    return f"Error: Tool {tool_idx} not implemented in fallback mode."


async def write_script_async(content: str, path: str) -> str:
    """Async wrapper for writing a script."""
    return await _run_env_step(0, content)


async def edit_script_async(path: str, find: str, replace: str) -> str:
    """Async wrapper for editing a script."""
    return await _run_env_step(1, f"{find}|||{replace}")


async def preview_design_async(path: str) -> str:
    """Async wrapper for previewing a design."""
    return await _run_env_step(2, path)


async def search_docs_async(query: str) -> str:
    """Async wrapper for searching documentation."""
    return await _run_env_step(3, query)


async def search_parts_async(query: str) -> str:
    """Async wrapper for searching COTS parts."""
    return await _run_env_step(5, query)


async def preview_part_async(part_id: str) -> str:
    """Async wrapper for previewing a COTS part."""
    return await _run_env_step(6, part_id)


async def check_manufacturability_async(
    design_file: str, process: str, quantity: int
) -> dict:
    """Async wrapper for check_manufacturability."""
    args = f"{process}|||{quantity}"
    output = await _run_env_step(7, args)
    try:
        import json

        return json.loads(output)
    except:
        return {"output": output}


async def submit_design_async(control_path: str) -> str:
    """Async wrapper for submitting a design."""
    return await _run_env_step(4, control_path)


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


async def run_skill_script_async(
    skill_name: str, script_name: str, arguments: str = ""
) -> str:
    """Async wrapper for running a skill script."""
    return await asyncio.to_thread(
        env_tools.run_skill_script, skill_name, script_name, arguments
    )


async def read_script_async(path: str) -> str:
    """Async wrapper for reading a script."""
    return await asyncio.to_thread(env_tools.read_script, path)
