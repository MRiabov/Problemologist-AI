import os
import uuid

import httpx
import pytest

from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from shared.enums import AgentName

CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")
...


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_071_filesystem_policy_precedence_and_reviewer_scope():
    """INT-071: Agents config precedence + reviewer write scope."""
    session_id = f"INT-071-{uuid.uuid4().hex[:8]}"

    async with httpx.AsyncClient(timeout=30.0) as client:
        create_resp = await client.post(
            f"{CONTROLLER_URL}/api/test/episodes",
            json={
                "task": "INT-071 filesystem policy",
                "session_id": session_id,
                "agent_name": AgentName.ENGINEER_CODER,
            },
        )
        assert create_resp.status_code == 201

    reviewer_client = WorkerClient(
        base_url=WORKER_LIGHT_URL,
        session_id=session_id,
        heavy_url=WORKER_HEAVY_URL,
    )
    reviewer_path_by_role = {
        AgentName.BENCHMARK_PLAN_REVIEWER: "reviews/benchmark-plan-review-round-1.md",
        AgentName.ENGINEER_PLAN_REVIEWER: "reviews/engineering-plan-review-round-1.md",
        AgentName.ENGINEER_EXECUTION_REVIEWER: "reviews/engineering-execution-review-round-1.md",
        AgentName.BENCHMARK_REVIEWER: "reviews/benchmark-review-round-1.md",
        AgentName.ELECTRONICS_REVIEWER: "reviews/electronics-review-round-1.md",
    }
    reviewer_fs_by_role = {
        role: RemoteFilesystemMiddleware(reviewer_client, agent_role=role)
        for role in reviewer_path_by_role
    }

    coder_client = WorkerClient(
        base_url=WORKER_LIGHT_URL,
        session_id=session_id,
        heavy_url=WORKER_HEAVY_URL,
    )
    coder_fs = RemoteFilesystemMiddleware(
        coder_client,
        agent_role=AgentName.ENGINEER_CODER,
    )
    # Reviewer write scope: reviewer-specific stage files only.
    for role, fs in reviewer_fs_by_role.items():
        assert await fs.write_file(reviewer_path_by_role[role], "Approved")
        with pytest.raises(PermissionError):
            await fs.write_file("plan.md", "forbidden")
        with pytest.raises(PermissionError):
            await fs.write_file("reviews/review-round-1/review.md", "forbidden")

    # Agent override over defaults (defaults deny all writes, coder allows script.py)
    assert await coder_fs.write_file("script.py", "print('ok')")
    assert await coder_fs.write_file("/validate_benchmark.py", "print('ok')")
    assert await coder_fs.write_file("/workspace/compat_alias.py", "print('ok')")

    # Deny > allow precedence for coder (**/*.py allowed, reviews/** denied)
    with pytest.raises(PermissionError):
        await coder_fs.write_file("reviews/engineering-plan-review-round-2.md", "x")

    # Unmatched path is denied
    with pytest.raises(PermissionError):
        await coder_fs.write_file("unmatched.txt", "denied")

    # System-only manifests: denied to all agent roles
    manifest_path = ".manifests/engineering_plan_review_manifest.json"
    with pytest.raises(PermissionError):
        await coder_fs.read_file(manifest_path)
    with pytest.raises(PermissionError):
        await coder_fs.write_file(manifest_path, "{}")
    with pytest.raises(PermissionError):
        await reviewer_fs_by_role[AgentName.ENGINEER_PLAN_REVIEWER].read_file(
            manifest_path
        )
    with pytest.raises(PermissionError):
        await reviewer_fs_by_role[AgentName.ENGINEER_PLAN_REVIEWER].write_file(
            manifest_path, "{}"
        )

    # Defense-in-depth filtering: list_files should not leak unreadable entry names.
    async with httpx.AsyncClient(timeout=30.0) as client:
        seed_allowed = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "plan.md", "content": "visible", "overwrite": True},
            headers={"X-Session-ID": session_id},
        )
        assert seed_allowed.status_code == 200
        seed_blocked = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "unmatched.txt", "content": "hidden", "overwrite": True},
            headers={"X-Session-ID": session_id},
        )
        assert seed_blocked.status_code == 200

    listed_paths = {entry.path for entry in await coder_fs.list_files("/")}
    assert "/plan.md" in listed_paths
    assert "/unmatched.txt" not in listed_paths

    workspace_alias_paths = {
        entry.path for entry in await coder_fs.list_files("/workspace")
    }
    assert "/plan.md" in workspace_alias_paths

    grep_matches = await coder_fs.grep("hidden", path="/")
    assert all(match.path != "/unmatched.txt" for match in grep_matches)


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_072_plan_refusal_validation_and_routing():
    """INT-072: plan_refusal.md validation + reviewer routing."""
    session_id = f"INT-072-{uuid.uuid4().hex[:8]}"

    async with httpx.AsyncClient(timeout=30.0) as client:
        create_resp = await client.post(
            f"{CONTROLLER_URL}/api/test/episodes",
            json={
                "task": "INT-072 plan refusal validation",
                "session_id": session_id,
                "agent_name": AgentName.ENGINEER_CODER,
            },
        )
        assert create_resp.status_code == 201
        episode_id = create_resp.json()["episode_id"]

        invalid_refusal = (
            "---\n"
            "reasons: [INVALID_REASON]\n"
            f"role: {AgentName.ENGINEER_CODER.value}\n"
            "---\n"
            "Evidence body\n"
        )
        write_invalid = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={"path": "plan_refusal.md", "content": invalid_refusal},
            headers={"X-Session-ID": session_id},
        )
        assert write_invalid.status_code == 200

        confirm_payload = {
            "review_content": "---\n"
            "decision: CONFIRM_PLAN_REFUSAL\n"
            "comments: [Confirmed]\n"
            "---\n"
            "Reviewer evidence"
        }
        invalid_confirm = await client.post(
            f"{CONTROLLER_URL}/api/episodes/{episode_id}/review",
            json=confirm_payload,
        )
        assert invalid_confirm.status_code == 422
        assert "Invalid plan_refusal.md" in invalid_confirm.text

        valid_refusal = (
            "---\n"
            "reasons: [PHYSICALLY_IMPOSSIBLE]\n"
            f"role: {AgentName.ENGINEER_CODER.value}\n"
            "---\n"
            "Detailed geometric evidence\n"
        )
        write_valid = await client.post(
            f"{WORKER_LIGHT_URL}/fs/write",
            json={
                "path": "plan_refusal.md",
                "content": valid_refusal,
                "overwrite": True,
            },
            headers={"X-Session-ID": session_id},
        )
        assert write_valid.status_code == 200

        valid_confirm = await client.post(
            f"{CONTROLLER_URL}/api/episodes/{episode_id}/review",
            json=confirm_payload,
        )
        assert valid_confirm.status_code == 200
        assert valid_confirm.json()["decision"] == "CONFIRM_PLAN_REFUSAL"

        reject_payload = {
            "review_content": "---\n"
            "decision: REJECT_PLAN_REFUSAL\n"
            "comments: [Plan is still viable]\n"
            "---\n"
            "Reviewer override rationale"
        }
        reject_resp = await client.post(
            f"{CONTROLLER_URL}/api/episodes/{episode_id}/review",
            json=reject_payload,
        )
        assert reject_resp.status_code == 200
        assert reject_resp.json()["decision"] == "REJECT_PLAN_REFUSAL"


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_073_observability_linkage():
    """INT-073: user_session_id -> episode_id -> child IDs linkage is observable."""
    user_session_id = uuid.uuid4()
    session_id = f"INT-073-{uuid.uuid4().hex[:8]}"

    async with httpx.AsyncClient(timeout=30.0) as client:
        create_resp = await client.post(
            f"{CONTROLLER_URL}/api/test/episodes",
            json={
                "task": "INT-073 observability linkage",
                "session_id": session_id,
                "user_session_id": str(user_session_id),
                "agent_name": AgentName.ENGINEER_CODER,
            },
        )
        assert create_resp.status_code == 201
        episode_id = create_resp.json()["episode_id"]

        review_payload = {
            "review_content": "---\n"
            "decision: APPROVED\n"
            "comments: [Looks good]\n"
            "---\n"
            "Evidence body"
        }
        review_resp = await client.post(
            f"{CONTROLLER_URL}/api/episodes/{episode_id}/review",
            json=review_payload,
        )
        assert review_resp.status_code == 200

        episode_resp = await client.get(f"{CONTROLLER_URL}/api/episodes/{episode_id}")
        assert episode_resp.status_code == 200
        episode_data = episode_resp.json()

        assert episode_data["user_session_id"] == str(user_session_id)

        review_trace = next(
            (
                trace
                for trace in episode_data.get("traces", [])
                if trace.get("name") == "review_decision"
            ),
            None,
        )
        assert review_trace is not None
        assert review_trace["user_session_id"] == str(user_session_id)
        assert review_trace["review_id"]
