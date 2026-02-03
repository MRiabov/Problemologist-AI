import asyncio
from typing import Any, Optional

# Remove direct import of tools module to force usage of Runtime
# from src.environment import tools as env_tools
from src.environment.runtime import ToolRuntime

# Global reference to the active environment (if any)
_ACTIVE_ENV: Any | None = None
_CURRENT_ROLE: str | None = None

# Runtime Registry
_RUNTIMES: dict[str, ToolRuntime] = {}
_DEFAULT_RUNTIME_ID: str = "default"

# Fallback runtime for when no env is active
_FALLBACK_RUNTIME: ToolRuntime | None = None


def _get_fallback_runtime() -> ToolRuntime:
    global _FALLBACK_RUNTIME
    if _FALLBACK_RUNTIME is None:
        _FALLBACK_RUNTIME = ToolRuntime("workspace")
    return _FALLBACK_RUNTIME


def register_runtime(runtime_id: str, runtime: ToolRuntime):
    """Registers a runtime instance."""
    global _RUNTIMES
    _RUNTIMES[runtime_id] = runtime


def get_runtime(runtime_id: str | None = None) -> ToolRuntime:
    """Gets a registered runtime by ID, or fallback."""
    if runtime_id and runtime_id in _RUNTIMES:
        return _RUNTIMES[runtime_id]

    # Legacy/Fallback priority:
    # 1. _ACTIVE_ENV.runtime (if set)
    # 2. _RUNTIMES["default"] (if set)
    # 3. _FALLBACK_RUNTIME

    if _ACTIVE_ENV and hasattr(_ACTIVE_ENV, "runtime"):
        return _ACTIVE_ENV.runtime

    if _DEFAULT_RUNTIME_ID in _RUNTIMES:
        return _RUNTIMES[_DEFAULT_RUNTIME_ID]

    return _get_fallback_runtime()


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


def _get_runtime_from_args(kwargs: dict) -> tuple[ToolRuntime, dict]:
    """Extracts runtime from kwargs if present, returns (runtime, cleaned_kwargs)."""
    runtime = None
    if "tool_runtime" in kwargs:
        runtime = kwargs.pop("tool_runtime")

    # If not passed explicitly, lookup via ID if passed (unlikely for tools, but possible)
    # or fallback to global context
    if runtime is None:
        runtime = get_runtime()

    return runtime, kwargs


async def _run_env_step(tool_name: str, **kwargs) -> str:
    """Helper to run a step in the active environment or fallback to direct tool call."""
    global _ACTIVE_ENV, _CURRENT_ROLE

    # Check for runtime in kwargs (injected)
    runtime_arg, kwargs = _get_runtime_from_args(kwargs.copy())

    # Check if arguments are passed as a single string (legacy fallback or LLM quirks)

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

    # Use runtime directly if available and no ACTIVE_ENV (or if we prefer runtime)
    # The requirement is to move away from implicit global state.
    # If runtime_arg was passed explicitly (injected), use it!
    # If not, checks ACTIVE_ENV for backward compatibility.

    if runtime_arg and runtime_arg is not _get_fallback_runtime():
         # If we resolved a specific runtime (not just the fallback), use it directly.
         # This bypasses CADEnv.step() logging if we are running "stateless tools" directly against runtime.
         # However, CADEnv.step logging is useful.
         # If we want to maintain logging, we might need to hook into it.
         # For now, we follow the instruction: "Stateless Tools".
         return await asyncio.to_thread(runtime_arg.dispatch, tool_name, kwargs)

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
    return await asyncio.to_thread(runtime_arg.dispatch, tool_name, kwargs)


async def write_file_async(content: str, path: str, mode: str = "overwrite", tool_runtime: Any = None) -> str:
    """Async wrapper for writing a file."""
    return await _run_env_step("write_file", content=content, path=path, mode=mode, tool_runtime=tool_runtime)


async def edit_file_async(path: str, find: str, replace: str, tool_runtime: Any = None) -> str:
    """Async wrapper for editing a file."""
    return await _run_env_step("edit_file", path=path, find=find, replace=replace, tool_runtime=tool_runtime)


async def write_script_async(content: str, path: str, tool_runtime: Any = None) -> str:
    """[DEPRECATED] Async wrapper for writing a script."""
    return await write_file_async(content, path, mode="overwrite", tool_runtime=tool_runtime)


async def edit_script_async(path: str, find: str, replace: str, tool_runtime: Any = None) -> str:
    """[DEPRECATED] Async wrapper for editing a script."""
    return await edit_file_async(path, find, replace, tool_runtime=tool_runtime)


async def preview_design_async(path: str, tool_runtime: Any = None) -> str:
    """Async wrapper for previewing a design."""
    return await _run_env_step("preview_design", filename=path, tool_runtime=tool_runtime)


async def search_docs_async(query: str, tool_runtime: Any = None) -> str:
    """Async wrapper for searching documentation."""
    return await _run_env_step("search_docs", query=query, tool_runtime=tool_runtime)


async def search_parts_async(query: str, tool_runtime: Any = None) -> str:
    """Async wrapper for searching COTS parts."""
    return await _run_env_step("search_parts", query=query, tool_runtime=tool_runtime)


async def preview_part_async(part_id: str, tool_runtime: Any = None) -> str:
    """Async wrapper for previewing a COTS part."""
    return await _run_env_step("preview_part", part_id=part_id, tool_runtime=tool_runtime)


async def check_manufacturability_async(
    design_file: str, process: str, quantity: int, tool_runtime: Any = None
) -> dict:
    """Async wrapper for check_manufacturability."""
    # env step returns string, we parse it
    output = await _run_env_step(
        "check_manufacturability",
        design_file=design_file,
        process=process,
        quantity=quantity,
        tool_runtime=tool_runtime
    )
    try:
        import json

        return json.loads(output)
    except:
        return {"output": output}


async def submit_design_async(control_path: str, tool_runtime: Any = None) -> str:
    """Async wrapper for submitting a design."""
    return await _run_env_step("submit_design", value=control_path, tool_runtime=tool_runtime)


async def read_skill_async(
    skill_name: str, filename: str, resource_type: Optional[str] = None, tool_runtime: Any = None
) -> str:
    """Async wrapper for reading a skill."""
    rt = tool_runtime if tool_runtime else get_runtime()
    return await asyncio.to_thread(
        rt.read_skill, skill_name, filename, resource_type
    )


async def list_skills_async(tool_runtime: Any = None) -> str:
    """Async wrapper for listing skills."""
    rt = tool_runtime if tool_runtime else get_runtime()
    return await asyncio.to_thread(rt.list_skills)


async def list_skill_files_async(skill_name: str, tool_runtime: Any = None) -> str:
    """Async wrapper for listing skill files."""
    rt = tool_runtime if tool_runtime else get_runtime()
    return await asyncio.to_thread(rt.list_skill_files, skill_name)


async def update_skill_async(
    skill_name: str,
    content: str,
    filename: str,
    resource_type: Optional[str] = None,
    tool_runtime: Any = None,
) -> str:
    """Async wrapper for updating a skill."""
    rt = tool_runtime if tool_runtime else get_runtime()
    return await asyncio.to_thread(
        rt.update_skill, skill_name, content, filename, resource_type
    )


async def init_skill_async(skill_name: str, tool_runtime: Any = None) -> str:
    """Async wrapper for initializing a skill."""
    rt = tool_runtime if tool_runtime else get_runtime()
    return await asyncio.to_thread(rt.init_skill, skill_name)


async def package_skill_async(skill_name: str, tool_runtime: Any = None) -> str:
    """Async wrapper for packaging a skill."""
    rt = tool_runtime if tool_runtime else get_runtime()
    return await asyncio.to_thread(rt.package_skill, skill_name)


async def run_skill_script_async(
    skill_name: str, script_name: str, arguments: str = "", tool_runtime: Any = None
) -> str:
    """Async wrapper for running a skill script."""
    rt = tool_runtime if tool_runtime else get_runtime()
    return await asyncio.to_thread(
        rt.run_skill_script, skill_name, script_name, arguments
    )


async def read_script_async(path: str, tool_runtime: Any = None) -> str:
    """Async wrapper for reading a script."""
    rt = tool_runtime if tool_runtime else get_runtime()
    return await asyncio.to_thread(rt.read_script, path)


async def view_file_async(path: str, tool_runtime: Any = None) -> str:
    """Async wrapper for viewing a file."""
    rt = tool_runtime if tool_runtime else get_runtime()
    return await asyncio.to_thread(rt.view_file, path)


async def run_command_async(command: str, tool_runtime: Any = None) -> str:
    """Async wrapper for running a command."""
    rt = tool_runtime if tool_runtime else get_runtime()
    return await asyncio.to_thread(rt.run_command, command)


async def start_session_async(session_id: str, tool_runtime: Any = None) -> str:
    """Async wrapper for starting a session."""
    rt = tool_runtime if tool_runtime else get_runtime()
    return await asyncio.to_thread(rt.start_session, session_id)


async def stop_session_async(tool_runtime: Any = None) -> str:
    """Async wrapper for stopping a session."""
    rt = tool_runtime if tool_runtime else get_runtime()
    return await asyncio.to_thread(rt.stop_session)


async def lint_script_async(filename: str, tool_runtime: Any = None) -> str:
    """Async wrapper for linting a script."""
    rt = tool_runtime if tool_runtime else get_runtime()
    return await asyncio.to_thread(rt.lint_script, filename)
