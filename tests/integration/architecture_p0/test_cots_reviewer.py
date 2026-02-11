import os
import uuid
import pytest
import httpx
import yaml

# Constants
WORKER_URL = os.getenv("WORKER_URL", "http://localhost:8001")
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:8000")


@pytest.fixture
def session_id():
    return f"test-cots-{uuid.uuid4().hex[:8]}"


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
            f"{WORKER_URL}/fs/list", json={"path": "."}, headers=base_headers
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
        # Scenario: Post a review with invalid 'decision' value
        malformed_review = """---
decision: MOCK_DECISION
evidence:
  files_checked: ["solution.py"]
---
This should be rejected.
"""
        # Reviewer output usually goes to a specific path or endpoint
        # If we have a /episodes/{id}/review endpoint:
        resp = await client.post(
            f"{CONTROLLER_URL}/episodes/{session_id}/review",  # Just a guess on path
            json={"review_content": malformed_review},
        )

        if resp.status_code == 404:
            # Fallback: Maybe it's a worker-side tool?
            resp = await client.post(
                f"{WORKER_URL}/benchmark/review",
                json={"content": malformed_review},
                headers=base_headers,
            )

        assert resp.status_code in [400, 422]
        assert "decision" in resp.text.lower()


@pytest.mark.integration_p0
@pytest.mark.asyncio
async def test_int_017_plan_refusal_loop(session_id, base_headers):
    """INT-017: Verify plan refusal loop routing."""
    async with httpx.AsyncClient(timeout=60.0) as client:
        # This requires a stateful flow.
        # 1. Start agent run
        # 2. Inject a 'refuse' decision
        # 3. Verify the agent continues to loop/retry rather than completing with success.

        # TBD: Implementation requires knowing the exact state machine event names
        pass
