import asyncio
import os
import uuid
from pathlib import Path

import httpx
import pytest
from pydantic import TypeAdapter

from controller.agent.prompt_manager import PromptManager
from controller.api.schemas import (
    AgentRunRequest,
    AgentRunResponse,
    EpisodeResponse,
)
from shared.enums import AgentName, EpisodeStatus
from shared.skills import build_skill_catalog_lines
from shared.skills.catalog import iter_skill_catalog_entries
from shared.workers.filesystem.backend import FileInfo
from shared.workers.schema import ListFilesRequest, ReadFileRequest, ReadFileResponse
from tests.integration.agent.helpers import (
    seed_benchmark_assembly_definition,
    seed_engineer_planner_handover,
)

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
ROOT = Path(__file__).resolve().parents[3]


def _catalog_paths(prompt_text: str) -> list[str]:
    catalog_block = prompt_text.rsplit("Available skills you can read:", 1)[1]
    paths: list[str] = []
    for line in catalog_block.splitlines():
        line = line.strip()
        if not line.startswith("- `"):
            continue
        paths.append(line.split("`")[1])
    return paths


def _snapshot_tree(root: Path) -> tuple[set[str], dict[str, bytes]]:
    dirs: set[str] = set()
    snapshot: dict[str, bytes] = {}
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [name for name in dirnames if name != ".git"]
        rel_dir = Path(dirpath).relative_to(root).as_posix()
        if rel_dir != ".":
            dirs.add(rel_dir)
        for filename in filenames:
            if filename == ".git":
                continue
            path = Path(dirpath, filename)
            rel_path = path.relative_to(root).as_posix()
            snapshot[rel_path] = path.read_bytes()
    return dirs, snapshot


@pytest.mark.integration_p1
@pytest.mark.asyncio
async def test_int_045_skills_sync_lifecycle():
    """INT-045: Verify skills sync lifecycle."""
    async with httpx.AsyncClient(timeout=30.0) as client:
        # 1. Trigger Agent Run
        task = "Write a script that uses a skill"
        session_id = f"INT-045-{uuid.uuid4().hex[:8]}"
        await seed_engineer_planner_handover(
            client,
            session_id=session_id,
            int_id="INT-045",
        )
        await seed_benchmark_assembly_definition(client, session_id)
        request = AgentRunRequest(
            task=task,
            session_id=session_id,
        )
        resp = await client.post(
            f"{CONTROLLER_URL}/agent/run",
            json=request.model_dump(),
        )
        assert resp.status_code == 202
        run_data = AgentRunResponse.model_validate(resp.json())
        episode_id = run_data.episode_id

        # 2. Wait for it to start/complete
        max_retries = 30
        for _ in range(max_retries):
            status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
            if status_resp.status_code == 200:
                episode_data = EpisodeResponse.model_validate(status_resp.json())
                if episode_data.status in [
                    EpisodeStatus.COMPLETED,
                    EpisodeStatus.FAILED,
                ]:
                    break
            await asyncio.sleep(1)

        # 3. Verify skills are present on the worker
        # Agent initialization copies skills to /skills directory on worker
        fs_ls_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/ls",
            json=ListFilesRequest(path="/skills").model_dump(),
            headers={"X-Session-ID": str(request.session_id)},
        )
        assert fs_ls_resp.status_code == 200
        files = TypeAdapter(list[FileInfo]).validate_python(fs_ls_resp.json())
        assert len(files) > 0

        # Check for a specific known skill
        assert any("build123d-cad-drafting-skill" in f.name for f in files)

        # 4. Verify skill content is readable
        skill_path = "/skills/build123d-cad-drafting-skill/SKILL.md"
        fs_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/read",
            json=ReadFileRequest(path=skill_path).model_dump(),
            headers={"X-Session-ID": str(request.session_id)},
        )
        assert fs_resp.status_code == 200
        content_data = ReadFileResponse.model_validate(fs_resp.json())
        assert "# Build123d CAD Drafting Skill" in content_data.content

        # 5. Verify the controller API and prompt builder expose the same checked-in
        # skill catalog entries and exclude non-skill directories such as `.git`.
        skills_resp = await client.get(f"{CONTROLLER_URL}/skills/")
        assert skills_resp.status_code == 200
        skills_payload = skills_resp.json()
        expected_skill_entries = list(iter_skill_catalog_entries())
        assert [item["name"] for item in skills_payload] == [
            name for name, _ in expected_skill_entries
        ]
        assert [item["description"] for item in skills_payload] == [
            description for _, description in expected_skill_entries
        ]
        assert all(item["name"] != ".git" for item in skills_payload)

        # 6. Verify the controller backend prompt builder exposes the skill catalog.
        prompt_manager = PromptManager()
        expected_catalog_paths = sorted(
            f".agents/skills/{skill_dir.name}/SKILL.md"
            for skill_dir in (ROOT / ".agents" / "skills").iterdir()
            if skill_dir.is_dir() and (skill_dir / "SKILL.md").is_file()
        )
        catalog_lines = build_skill_catalog_lines()
        assert catalog_lines[0] == "Available skills you can read:"
        assert _catalog_paths("\n".join(catalog_lines)) == expected_catalog_paths
        for agent_name in (AgentName.ENGINEER_CODER, AgentName.BENCHMARK_CODER):
            prompt_text = prompt_manager.render(agent_name)
            assert "Available skills you can read:" in prompt_text
            assert _catalog_paths(prompt_text) == expected_catalog_paths

        # 7. Verify the checked-in skill mirrors stay equal to the canonical tree.
        canonical_tree = _snapshot_tree(ROOT / ".agents" / "skills")
        for mirror_root in (
            ROOT / "skills",
            ROOT / ".codex" / "skills",
        ):
            assert _snapshot_tree(mirror_root) == canonical_tree
