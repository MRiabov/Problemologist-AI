import asyncio
from typing import Any, Optional

# Remove direct import of tools module to force usage of Runtime
# from src.environment import tools as env_tools
from src.environment.runtime import ToolRuntime

# Global reference to the active environment (if any)
_ACTIVE_ENV: Any | None = None
_CURRENT_ROLE: str | None = None

# Fallback runtime for when no env is active
_FALLBACK_RUNTIME: ToolRuntime | None = None


def _get_fallback_runtime() -> ToolRuntime:
    global _FALLBACK_RUNTIME
    if _FALLBACK_RUNTIME is None:
        _FALLBACK_RUNTIME = ToolRuntime("workspace")
    return _FALLBACK_RUNTIME


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


def _get_runtime() -> ToolRuntime:
    """Helper to get the active runtime or fallback."""
    if _ACTIVE_ENV and hasattr(_ACTIVE_ENV, "runtime"):
        return _ACTIVE_ENV.runtime
    return _get_fallback_runtime()


async def _run_env_step(tool_name: str, **kwargs) -> str:
    """Helper to run a step in the active environment or fallback to direct tool call."""
    global _ACTIVE_ENV, _CURRENT_ROLE

    # Check if arguments are passed as a single string (legacy fallback or LLM quirks)
    if (
        "arguments" in kwargs
        and isinstance(kwargs["arguments"], str)
        and len(kwargs) == 1
    ):
        import json

        try:
            if kwargs["arguments"].strip().startswith("{"):
                kwargs = json.loads(kwargs["arguments"])
        except Exception:
            pass

    if _ACTIVE_ENV:
        import json

        action = {"tool": tool_name, "arguments": json.dumps(kwargs)}
        try:
            obs, reward, terminated, truncated, info = await asyncio.to_thread(
                _ACTIVE_ENV.step, action, agent_role=_CURRENT_ROLE
            )
            return obs["last_output"]
        except Exception as e:
            return f"Error executing tool {tool_name}: {e}"

    # Fallback to direct runtime dispatch
    runtime = _get_fallback_runtime()
    return await asyncio.to_thread(runtime.dispatch, tool_name, kwargs)


async def write_file_async(content: str, path: str, mode: str = "overwrite") -> str:
    """Async wrapper for writing a file."""
    return await _run_env_step("write_file", content=content, path=path, mode=mode)


async def edit_file_async(path: str, find: str, replace: str) -> str:
    """Async wrapper for editing a file."""
    return await _run_env_step("edit_file", path=path, find=find, replace=replace)


async def write_script_async(content: str, path: str) -> str:
    """[DEPRECATED] Async wrapper for writing a script."""
    return await write_file_async(content, path, mode="overwrite")


async def edit_script_async(path: str, find: str, replace: str) -> str:
    """[DEPRECATED] Async wrapper for editing a script."""
    return await edit_file_async(path, find, replace)


async def preview_design_async(path: str) -> str:
    """Async wrapper for previewing a design."""
    return await _run_env_step("preview_design", filename=path)


async def search_docs_async(query: str) -> str:
    """Async wrapper for searching documentation."""
    return await _run_env_step("search_docs", query=query)


async def search_parts_async(query: str) -> str:
    """Async wrapper for searching COTS parts."""
    return await _run_env_step("search_parts", query=query)


async def preview_part_async(part_id: str) -> str:
    """Async wrapper for previewing a COTS part."""
    return await _run_env_step("preview_part", part_id=part_id)


async def check_manufacturability_async(
    design_file: str, process: str, quantity: int
) -> dict:
    """Async wrapper for check_manufacturability."""
    # env step returns string, we parse it
    output = await _run_env_step(
        "check_manufacturability",
        design_file=design_file,
        process=process,
        quantity=quantity,
    )
    try:
        import json

        return json.loads(output)
    except:
        return {"output": output}


async def submit_design_async(control_path: str) -> str:
    """Async wrapper for submitting a design."""
    return await _run_env_step("submit_design", value=control_path)


async def read_skill_async(
    skill_name: str, filename: str, resource_type: Optional[str] = None
) -> str:
    """Async wrapper for reading a skill."""
    return await asyncio.to_thread(
        _get_runtime().read_skill, skill_name, filename, resource_type
    )


async def list_skills_async() -> str:
    """Async wrapper for listing skills."""
    return await asyncio.to_thread(_get_runtime().list_skills)


async def list_skill_files_async(skill_name: str) -> str:
    """Async wrapper for listing skill files."""
    return await asyncio.to_thread(_get_runtime().list_skill_files, skill_name)


async def update_skill_async(
    skill_name: str,
    content: str,
    filename: str,
    resource_type: Optional[str] = None,
) -> str:
    """Async wrapper for updating a skill."""
    return await asyncio.to_thread(
        _get_runtime().update_skill, skill_name, content, filename, resource_type
    )


async def init_skill_async(skill_name: str) -> str:
    """Async wrapper for initializing a skill."""
    return await asyncio.to_thread(_get_runtime().init_skill, skill_name)


async def package_skill_async(skill_name: str) -> str:
    """Async wrapper for packaging a skill."""
    return await asyncio.to_thread(_get_runtime().package_skill, skill_name)


async def run_skill_script_async(
    skill_name: str, script_name: str, arguments: str = ""
) -> str:
    """Async wrapper for running a skill script."""
    return await asyncio.to_thread(
        _get_runtime().run_skill_script, skill_name, script_name, arguments
    )


async def read_script_async(path: str) -> str:
    """Async wrapper for reading a script."""
    return await asyncio.to_thread(_get_runtime().read_script, path)


async def view_file_async(path: str) -> str:
    """Async wrapper for viewing a file."""
    return await asyncio.to_thread(_get_runtime().view_file, path)


async def run_command_async(command: str) -> str:
    """Async wrapper for running a command."""
    return await asyncio.to_thread(_get_runtime().run_command, command)


async def start_session_async(session_id: str) -> str:
    """Async wrapper for starting a session."""
    return await asyncio.to_thread(_get_runtime().start_session, session_id)


async def stop_session_async() -> str:
    """Async wrapper for stopping a session."""
    return await asyncio.to_thread(_get_runtime().stop_session)


async def lint_script_async(filename: str) -> str:
    """Async wrapper for linting a script."""
    return await asyncio.to_thread(_get_runtime().lint_script, filename)
