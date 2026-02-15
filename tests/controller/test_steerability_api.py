import pytest
from fastapi.testclient import TestClient
from controller.api.main import app
from controller.services.steerability.service import steerability_service
from shared.models.steerability import SteerablePrompt

client = TestClient(app)


@pytest.fixture(autouse=True)
def clear_queues():
    # Clear queues before each test
    steerability_service._queues = {}


def test_steer_endpoint_enqueues():
    session_id = "test_session_1"
    prompt_data = {
        "text": "Move this part 10mm up",
        "selections": [{"level": "PART", "target_id": "part_A", "center": [0, 0, 0]}],
    }

    response = client.post(f"/api/v1/sessions/{session_id}/steer", json=prompt_data)
    assert response.status_code == 202
    assert response.json()["status"] == "queued"
    assert response.json()["queue_position"] == 1

    # Verify it's in the queue
    response = client.get(f"/api/v1/sessions/{session_id}/queue")
    assert response.status_code == 200
    queue = response.json()
    assert len(queue) == 1
    assert queue[0]["text"] == "Move this part 10mm up"


def test_multiple_prompts_ordering():
    session_id = "test_session_2"
    client.post(f"/api/v1/sessions/{session_id}/steer", json={"text": "Prompt 1"})
    client.post(f"/api/v1/sessions/{session_id}/steer", json={"text": "Prompt 2"})

    response = client.get(f"/api/v1/sessions/{session_id}/queue")
    queue = response.json()
    assert len(queue) == 2
    assert queue[0]["text"] == "Prompt 1"
    assert queue[1]["text"] == "Prompt 2"


def test_session_isolation():
    client.post("/api/v1/sessions/s1/steer", json={"text": "Prompt S1"})
    client.post("/api/v1/sessions/s2/steer", json={"text": "Prompt S2"})

    res1 = client.get("/api/v1/sessions/s1/queue")
    assert len(res1.json()) == 1
    assert res1.json()[0]["text"] == "Prompt S1"

    res2 = client.get("/api/v1/sessions/s2/queue")
    assert len(res2.json()) == 1
    assert res2.json()[0]["text"] == "Prompt S2"


@pytest.mark.asyncio
async def test_service_dequeue():
    session_id = "service_test"
    prompt = SteerablePrompt(text="Direct Service Test")
    await steerability_service.enqueue_prompt(session_id, prompt)

    dequeued = await steerability_service.dequeue_prompt(session_id)
    assert dequeued.text == "Direct Service Test"

    # Queue should be empty now
    queue = await steerability_service.get_queued_prompts(session_id)
    assert len(queue) == 0
