import asyncio
from pathlib import Path

from structlog import get_logger

from controller.clients.backend import RemoteFilesystemBackend
from shared.agent_templates import load_common_template_files
from shared.enums import AgentName

logger = get_logger(__name__)

TEMPLATE_REPOS_ROOT = (
    Path(__file__).resolve().parents[2] / "shared" / "assets" / "template_repos"
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


def _role_template_files(agent_name: AgentName) -> dict[str, str]:
    """Load role-specific starter files that are not common boilerplate."""
    role_mappings: dict[AgentName, dict[str, str]] = {
        AgentName.ENGINEER_PLANNER: {
            "engineer/plan.md": "plan.md",
            "engineer/todo.md": "todo.md",
            "engineer/assembly_definition.yaml": "assembly_definition.yaml",
        },
        AgentName.ELECTRONICS_PLANNER: {
            "engineer/plan.md": "plan.md",
            "engineer/todo.md": "todo.md",
            "engineer/assembly_definition.yaml": "assembly_definition.yaml",
        },
        AgentName.BENCHMARK_PLANNER: {
            "benchmark_generator/plan.md": "plan.md",
            "benchmark_generator/todo.md": "todo.md",
            "benchmark_generator/benchmark_definition.yaml": "benchmark_definition.yaml",
            "benchmark_generator/benchmark_assembly_definition.yaml": "benchmark_assembly_definition.yaml",
        },
    }

    mapping = role_mappings.get(agent_name)
    if mapping is None:
        return {}

    loaded: dict[str, str] = {}
    for template_path, remote_name in mapping.items():
        source_path = TEMPLATE_REPOS_ROOT / template_path
        if not source_path.is_file():
            raise FileNotFoundError(f"Template file not found: {source_path}")
        loaded[remote_name] = source_path.read_text(encoding="utf-8")
    return loaded


async def _write_template(
    backend: RemoteFilesystemBackend,
    template_name: str,
    template_content: str,
    target_path: str,
) -> None:
    """Write a local template payload to the remote worker."""
    try:
        await backend.awrite(target_path, template_content)
        logger.info("wrote_template", path=template_name, target=target_path)
    except Exception as e:
        logger.error(
            "failed_to_write_template",
            path=template_name,
            target=target_path,
            error=str(e),
            session_id=backend.client.session_id,
        )


def _can_write_template(
    backend: RemoteFilesystemBackend, agent_name: AgentName, target_path: str
) -> bool:
    policy = getattr(getattr(backend, "client", None), "policy", None)
    if policy is None:
        return True

    try:
        return bool(policy.check_permission(agent_name, "write", target_path))
    except Exception:
        return True


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

    tasks = []

    template_files = load_common_template_files()
    template_files.update(_role_template_files(agent_name))
    for remote_name, content in template_files.items():
        if not overwrite and _normalize_remote_path(remote_name) in existing_files:
            logger.info("skipping_existing_file", path=remote_name)
            continue
        if not _can_write_template(backend, agent_name, remote_name):
            logger.info("skipping_disallowed_template", path=remote_name)
            continue
        tasks.append(_write_template(backend, remote_name, content, remote_name))

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
