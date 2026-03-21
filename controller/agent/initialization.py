import asyncio

from structlog import get_logger

from controller.clients.backend import RemoteFilesystemBackend
from shared.agent_templates import load_agent_template_files
from shared.enums import AgentName

logger = get_logger(__name__)


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

    template_files = load_agent_template_files(agent_name)
    for remote_name, content in template_files.items():
        if not overwrite and _normalize_remote_path(remote_name) in existing_files:
            logger.info("skipping_existing_file", path=remote_name)
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
