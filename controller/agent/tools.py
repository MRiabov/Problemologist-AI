from collections.abc import Callable
from pathlib import Path

from controller.middleware.remote_fs import EditOp, RemoteFilesystemMiddleware
from controller.observability.tracing import record_worker_events
from shared.cots.agent import search_cots_catalog
from shared.observability.schemas import RunCommandToolEvent

SKILLS_DIR = Path(".agent/skills")


def get_common_tools(fs: RemoteFilesystemMiddleware, session_id: str) -> list[Callable]:
    """
    Get the set of common tools available to all agents (Engineer, Benchmark, etc.).
    Includes filesystem operations and COTS catalog search.
    """

    async def list_files(path: str = "/"):
        """List files in the workspace (filesystem)."""
        return await fs.list_files(path)

    async def read_file(path: str):
        """Read a file's content from the workspace."""
        return await fs.read_file(path)

    async def write_file(path: str, content: str, overwrite: bool = False):
        """Write content to a file in the workspace."""
        return await fs.write_file(path, content, overwrite=overwrite)

    async def edit_file(path: str, old_string: str, new_string: str):
        """Edit a file by replacing old_string with new_string."""
        return await fs.edit_file(
            path, [EditOp(old_string=old_string, new_string=new_string)]
        )

    async def grep(pattern: str, path: str | None = None, glob: str | None = None):
        """Search for a pattern in files."""
        return await fs.grep(pattern, path, glob)

    async def execute_command(command: str):
        """Execute a shell command in the workspace."""
        # Record the command execution event
        await record_worker_events(
            episode_id=session_id,
            events=[RunCommandToolEvent(command=command)],
        )
        return await fs.run_command(command)

    async def inspect_topology(target_id: str, script_path: str = "script.py") -> dict:
        """
        Inspect geometric properties of a selected feature (face, edge, part).
        Returns center, normal, area, and bounding box.
        """
        return await fs.inspect_topology(target_id, script_path)

    async def list_skill_files(skill_name: str) -> str:
        """List files in a specific skill directory."""
        skill_path = (SKILLS_DIR / skill_name).resolve()

        # Security check to prevent directory traversal
        try:
            skill_path.relative_to(SKILLS_DIR.resolve())
        except ValueError:
            return "Error: Access denied (path traversal attempt)."

        if not skill_path.exists() or not skill_path.is_dir():
            return f"Error: Skill '{skill_name}' not found."

        files = [
            f.name
            for f in skill_path.iterdir()
            if f.is_file() and not f.name.startswith(".")
        ]
        return "\n".join(sorted(files))

    async def read_skill(skill_name: str, filename: str) -> str:
        """Read a file from a specific skill directory."""
        # Check if skill exists first to avoid confusion
        if not (SKILLS_DIR / skill_name).exists():
            return f"Error: Skill '{skill_name}' not found."

        file_path = (SKILLS_DIR / skill_name / filename).resolve()

        # Security check to prevent directory traversal
        try:
            file_path.relative_to(SKILLS_DIR.resolve())
        except ValueError:
            return "Error: Access denied (path traversal attempt)."

        if not file_path.exists():
            return f"Error: File '{filename}' not found in skill '{skill_name}'."

        return file_path.read_text(encoding="utf-8")

    return [
        list_files,
        read_file,
        write_file,
        edit_file,
        grep,
        execute_command,
        inspect_topology,
        search_cots_catalog,
        list_skill_files,
        read_skill,
    ]


def get_engineer_tools(
    fs: RemoteFilesystemMiddleware, session_id: str
) -> list[Callable]:
    """
    Get the tools for the Engineer agent.
    Now uses the common toolset.
    """
    return get_common_tools(fs, session_id)
