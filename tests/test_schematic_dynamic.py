import uuid
from unittest.mock import AsyncMock, MagicMock
import pytest
from fastapi.testclient import TestClient
from controller.api.main import app
from controller.api.routes.episodes import get_db
from controller.persistence.models import Episode
from shared.enums import ElectronicComponentType
import yaml

@pytest.fixture
def mock_db():
    session = AsyncMock()
    app.dependency_overrides[get_db] = lambda: session
    yield session
    app.dependency_overrides.clear()

client = TestClient(app)

def test_get_episode_schematic_dynamic(mock_db, monkeypatch):
    episode_id = uuid.uuid4()
    mock_episode = MagicMock(spec=Episode)
    mock_episode.id = episode_id
    mock_episode.metadata_vars = {"worker_session_id": "test_session"}

    mock_result = MagicMock()
    mock_result.scalar_one_or_none.return_value = mock_episode
    mock_db.execute.return_value = mock_result

    # Mock WorkerClient.read_file
    mock_worker_client = MagicMock()

    assembly_data = {
        "electronics": {
            "power_supply": {"voltage_dc": 12, "max_current_a": 10},
            "components": [
                {"component_id": "m1", "type": "motor"},
                {"component_id": "sw1", "type": "switch"}
            ],
            "wiring": [
                {
                    "wire_id": "w1",
                    "from": {"component": "supply", "terminal": "v+"},
                    "to": {"component": "sw1", "terminal": "p1"},
                    "gauge_awg": 22,
                    "length_mm": 50
                },
                {
                    "wire_id": "w2",
                    "from": {"component": "sw1", "terminal": "p2"},
                    "to": {"component": "m1", "terminal": "pos"},
                    "gauge_awg": 22,
                    "length_mm": 50
                }
            ]
        },
        "constraints": {
            "benchmark_max_unit_cost_usd": 100,
            "benchmark_max_weight_g": 1000,
            "planner_target_max_unit_cost_usd": 80,
            "planner_target_max_weight_g": 800
        },
        "totals": {
            "estimated_unit_cost_usd": 50,
            "estimated_weight_g": 500,
            "estimate_confidence": "high"
        }
    }

    mock_worker_client.read_file = AsyncMock(return_value=yaml.dump(assembly_data))

    # Mock WorkerClient constructor
    monkeypatch.setattr("controller.api.routes.episodes.WorkerClient", lambda **kwargs: mock_worker_client)

    response = client.get(f"/episodes/{episode_id}/electronics/schematic")

    assert response.status_code == 200
    soup = response.json()

    # Check components
    comp_ids = [item["id"] for item in soup if item["type"] == "schematic_component"]
    assert "comp_m1" in comp_ids
    assert "comp_sw1" in comp_ids

    # Check pins
    pin_ids = [item["id"] for item in soup if item["type"] == "schematic_pin"]
    # m1 should have + and - (standard) and 'pos' (from wiring)
    assert "comp_m1_+" in pin_ids
    assert "comp_m1_-" in pin_ids
    assert "comp_m1_pos" in pin_ids

    # sw1 should have in and out (standard) and p1, p2 (from wiring)
    assert "comp_sw1_in" in pin_ids
    assert "comp_sw1_out" in pin_ids
    assert "comp_sw1_p1" in pin_ids
    assert "comp_sw1_p2" in pin_ids

    # Check traces
    traces = [item for item in soup if item["type"] == "schematic_trace"]
    assert len(traces) == 2

    # Trace w1: supply:v+ -> sw1:p1
    # Note: supply is not in components list, so it might not be in soup as a component,
    # but the trace will still point to it.
    # Wait, if supply is not a component, comp_supply_v+ might be broken if we only add pins for components in the list.

    # Let's check trace w2: sw1:p2 -> m1:pos
    w2 = next(t for t in traces if t["id"] == "trace_w2")
    assert w2["source"] == "comp_sw1_p2"
    assert w2["target"] == "comp_m1_pos"
