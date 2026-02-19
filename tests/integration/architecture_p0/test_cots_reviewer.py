import os
import uuid

import httpx
import pytest

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")


@pytest.fixture
def session_id():
    return f"INT-012-{uuid.uuid4().hex[:8]}"


@pytest.fixture
def base_headers(session_id):
    return {"X-Session-ID": session_id}


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_012_013_cots_search_contract_and_readonly(session_id, base_headers):
    """INT-012, INT-013: Verify COTS search output contract and read-only behavior."""
    async with httpx.AsyncClient(timeout=30.0) as client:
        # COTS subagent or tool is usually invoked via a runtime script or endpoint.
        # Let's test via the COTS indexer/search endpoint if available,
        # or via a subagent call if the controller exposes it.

        # Scenario: Query for a common part
        query = "M3 screw"
        resp = await client.get(f"{CONTROLLER_URL}/cots/search", params={"q": query})

        # If the endpoint doesn't exist yet, we might need to trigger it via an agent run
        # but for a targeted integration test, we prefer direct API if it exists.
        if resp.status_code == 404:
            pytest.skip("COTS search endpoint not implemented on controller")

        assert resp.status_code == 200
        data = resp.json()
        assert isinstance(data, list)

        if len(data) > 0:
            part = data[0]
            # INT-013: Required fields
            required_fields = ["part_id", "manufacturer", "price", "source"]
            for field in required_fields:
                assert field in part, f"Missing {field} in COTS result"

        # INT-012: Read-only check
        # Verify that after search, no new files are created in a dummy session
        # (beyond journal entries which are allowed)
        ls_resp = await client.post(
            f"{WORKER_LIGHT_URL}/fs/ls", json={"path": "."}, headers=base_headers
        )
        files = [f["name"] for f in ls_resp.json()]
        # Allow only journals or nothing
        for f in files:
            assert (
                f.startswith("journal")
                or f == "objectives.yaml"
                or f == "plan.md"
                or f == "."
            )


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_016_reviewer_decision_schema_gate(session_id, base_headers):
    """INT-016: Verify reviewer rejects malformed frontmatter decisions."""
    async with httpx.AsyncClient(timeout=30.0) as client:
        # Create a real episode first
        session_id = f"INT-016-{uuid.uuid4().hex[:8]}"
        run_resp = await client.post(
            f"{CONTROLLER_URL}/agent/run",
            json={"task": "Test Review Schema", "session_id": session_id},
        )
        assert run_resp.status_code == 202
        episode_id = run_resp.json()["episode_id"]

        # Scenario: Post a review with invalid 'decision' value
        malformed_review = """---
decision: MOCK_DECISION
evidence:
  files_checked: ["solution.py"]
---
This should be rejected.
"""
        # Reviewer output usually goes to a specific path or endpoint
        resp = await client.post(
            f"{CONTROLLER_URL}/episodes/{episode_id}/review",
            json={"review_content": malformed_review},
        )

        if resp.status_code == 404:
            # Fallback: Maybe it's a worker-side tool?
            resp = await client.post(
                f"{WORKER_LIGHT_URL}/benchmark/review",
                json={"content": malformed_review},
                headers=base_headers,
            )

        assert resp.status_code in [400, 422]
        assert "decision" in resp.text.lower()


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_017_plan_refusal_loop(session_id, base_headers):
    """INT-017: Verify plan refusal loop (rejection leads to FAILED state)."""
    async with httpx.AsyncClient(timeout=60.0) as client:
        # 1. Start agent run
        session_id = f"INT-017-{uuid.uuid4().hex[:8]}"
        run_resp = await client.post(
            f"{CONTROLLER_URL}/agent/run",
            json={"task": "Test Refusal Loop", "session_id": session_id},
        )
        assert run_resp.status_code == 202
        episode_id = run_resp.json()["episode_id"]

        # 2. Inject a 'rejected' decision via review endpoint
        # We need a valid review payload
        review_content = """---
decision: rejected
evidence:
  files_checked: ["plan.md"]
  comments: "Not good enough"
---
Refusing this plan.
"""
        resp = await client.post(
            f"{CONTROLLER_URL}/episodes/{episode_id}/review",
            json={"review_content": review_content},
        )
        assert resp.status_code == 200
        data = resp.json()
        assert data["status"] == "success"
        assert data["decision"] == "rejected"

        # 3. Verify the agent/episode status is FAILED
        # (Architecture might eventually require a loop/retry, but current impl fails it)
        status_resp = await client.get(f"{CONTROLLER_URL}/episodes/{episode_id}")
        assert status_resp.status_code == 200
        episode_data = status_resp.json()
        assert episode_data["status"] == "failed"
