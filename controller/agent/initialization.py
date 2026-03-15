import asyncio
from pathlib import Path

from structlog import get_logger

from controller.clients.backend import RemoteFilesystemBackend
from shared.enums import AgentName

logger = get_logger(__name__)

# Use relative path from this file to find shared assets
# controller/agent/initialization.py -> ../../shared/assets/template_repos
SHARED_ASSETS_PATH = (
    Path(__file__).resolve().parent.parent.parent / "shared/assets/template_repos"
)


def _normalize_remote_path(path: str) -> str:
    """Normalize worker paths so '/a/b.md' and 'a/b.md' compare equally."""
    return path.strip().lstrip("/").rstrip("/")


def _extract_entry_path(entry: object) -> str | None:
    """Extract a path string from ls entry payloads (dict or model-like)."""
    if isinstance(entry, dict):
        value = entry.get("path")
        return str(value) if value is not None else None
    value = getattr(entry, "path", None)
    return str(value) if value is not None else None


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
            session_id=backend.client.session_id,
        )


async def initialize_agent_files(
    backend: RemoteFilesystemBackend, agent_name: AgentName, overwrite: bool = False
) -> None:
    """
    Initializes the file system for a given agent with templates and directories.
    If overwrite=False (default), it skips files that already exist.
    """
    logger.info("initializing_agent_files", agent_name=agent_name)

    # 1. Check existing files to avoid overwriting pre-seeded evaluation data
    existing_files = set()
    try:
        # als_info(".") should list the root directory where these files go
        info = await backend.als_info(".")
        existing_files = {
            normalized
            for item in info
            if (path := _extract_entry_path(item))
            and (normalized := _normalize_remote_path(path))
        }
        logger.info("found_existing_files", count=len(existing_files))
    except Exception as e:
        logger.warning("failed_to_list_existing_files", error=str(e))

    # Define agent-specific file mappings
    # Format: local_template_subpath -> remote_path
    file_mappings = {}

    # Engineer Agents
    if agent_name == AgentName.ENGINEER_PLANNER:
        file_mappings = {
            "engineer/plan.md": "plan.md",
            "engineer/todo.md": "todo.md",
            "engineer/assembly_definition.yaml": "assembly_definition.yaml",
            "shared/journal.md": "journal.md",
        }
    elif agent_name == AgentName.ENGINEER_CODER:
        file_mappings = {
            "engineer/todo.md": "todo.md",
            "shared/journal.md": "journal.md",
        }

    # Benchmark Agents
    elif agent_name in [AgentName.BENCHMARK_PLANNER, AgentName.BENCHMARK_CODER]:
        file_mappings = {
            "benchmark_generator/plan.md": "plan.md",
            "benchmark_generator/todo.md": "todo.md",
            "benchmark_generator/benchmark_definition.yaml": "benchmark_definition.yaml",
            "benchmark_generator/benchmark_assembly_definition.yaml": "benchmark_assembly_definition.yaml",
            "shared/journal.md": "journal.md",
        }

    # Support Agents
    elif agent_name == AgentName.SKILL_AGENT:
        file_mappings = {
            "shared/journal.md": "journal.md",
        }
    elif agent_name == AgentName.COTS_SEARCH:
        file_mappings = {}

    tasks = []

    for template_subpath, remote_name in file_mappings.items():
        if not overwrite and _normalize_remote_path(remote_name) in existing_files:
            logger.info("skipping_existing_file", path=remote_name)
            continue

        template_path = SHARED_ASSETS_PATH / template_subpath
        tasks.append(_write_template(backend, template_path, remote_name))

    if tasks:
        await asyncio.gather(*tasks)

    # Skills are mounted read-only at /skills on worker side; initialization must not
    # attempt to write there through normal agent permissions.
    available_skills = 0
    if hasattr(backend, "als_info"):
        try:
            entries = await backend.als_info("/skills")
            available_skills = len(
                [entry for entry in entries if entry.get("is_dir", False)]
            )
        except Exception as e:
            logger.warning("failed_to_list_skills_mount", error=str(e))

    logger.info(
        "agent_files_initialized",
        agent_name=agent_name,
        file_count=len(tasks),
        skills_available=available_skills,
    )
