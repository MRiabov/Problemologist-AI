import asyncio
from pathlib import Path
from structlog import get_logger

from controller.clients.backend import RemoteFilesystemBackend

logger = get_logger(__name__)

# Use relative path from this file to find shared assets
# controller/agent/initialization.py -> ../../shared/assets/template_repos
SHARED_ASSETS_PATH = (
    Path(__file__).resolve().parent.parent.parent / "shared/assets/template_repos"
)


async def _write_template(
    backend: RemoteFilesystemBackend,
    template_path: Path,
    target_path: str,
) -> None:
    """Reads a local template and writes it to the remote worker."""
    if not template_path.exists():
        logger.warning(
            "template_not_found", path=str(template_path), target=target_path
        )
        return

    try:
        content = template_path.read_text(encoding="utf-8")
        await backend.awrite(target_path, content)
        logger.info("wrote_template", path=str(template_path), target=target_path)
    except Exception as e:
        logger.error(
            "failed_to_write_template",
            path=str(template_path),
            target=target_path,
            error=str(e),
        )


async def initialize_agent_files(
    backend: RemoteFilesystemBackend, agent_name: str
) -> None:
    """
    Initializes the file system for a given agent with templates and directories.
    """
    logger.info("initializing_agent_files", agent_name=agent_name)

    # Define agent-specific file mappings
    # Format: local_template_subpath -> remote_path
    file_mappings = {}

    # Engineer Agents
    if agent_name in ["engineer_planner", "engineer_coder"]:
        file_mappings = {
            "engineer/plan.md": "plan.md",
            "engineer/todo.md": "todo.md",
            "shared/journal.md": "journal.md",
        }

    # Benchmark Agents
    elif agent_name in ["benchmark_planner", "benchmark_coder"]:
        file_mappings = {
            "benchmark_generator/plan.md": "plan.md",
            "benchmark_generator/todo.md": "todo.md",
            # Only planner gets objectives from template?
            # Or coder too? Coder needs to read it.
            # Planner generates it, but template might be useful as starting point.
            "benchmark_generator/objectives.yaml": "objectives.yaml",
            "shared/journal.md": "journal.md",
        }

    # Support Agents
    elif agent_name in ["cots_search", "skill_creator"]:
        file_mappings = {
            "shared/journal.md": "journal.md",
        }

    tasks = []

    for template_subpath, remote_name in file_mappings.items():
        template_path = SHARED_ASSETS_PATH / template_subpath
        tasks.append(_write_template(backend, template_path, remote_name))

    if tasks:
        await asyncio.gather(*tasks)

    logger.info("agent_files_initialized", agent_name=agent_name, file_count=len(tasks))
