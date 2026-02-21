from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient
from shared.workers.workbench_models import WorkbenchResult, WorkbenchMetadata
from shared.enums import ManufacturingMethod

from worker_heavy.app import app

client = TestClient(app)

@patch("worker_heavy.api.routes.create_filesystem_router")
@patch("worker_heavy.api.routes.load_component_from_script")
@patch("worker_heavy.utils.topology.validate_and_price")
def test_analyze_passes_method_and_quantity(mock_validate_and_price, mock_load, mock_get_router):
    # Setup mocks
    mock_load.return_value = MagicMock()
    mock_router = MagicMock()
    from pathlib import Path
    mock_router.local_backend.root = Path("/tmp")
    mock_get_router.return_value = mock_router

    mock_result = WorkbenchResult(
        is_manufacturable=True,
        unit_cost=10.0,
        weight_g=100.0,
        violations=[],
        metadata=WorkbenchMetadata(),
    )
    mock_validate_and_price.return_value = mock_result

    # Send request with specific method (3dp) and quantity (50)
    response = client.post(
        "/benchmark/analyze",
        json={
            "script_path": "impl.py",
            "method": "3dp",
            "quantity": 50
        },
        headers={"X-Session-ID": "test-session"}
    )

    assert response.status_code == 200

    # Check if validate_and_price was called with the correct arguments
    # The current implementation will fail here because it ignores method and quantity
    # causing method to default to CNC (or metadata based) and quantity to default to 1 (or whatever default)

    # We expect method=ManufacturingMethod.THREE_DP and quantity=50
    # Note: validate_and_price also takes config, so we can't assert strict equality on all args easily without capturing calls

    calls = mock_validate_and_price.call_args_list
    assert len(calls) == 1

    args, kwargs = calls[0]

    # Check kwargs for method and quantity
    # If passed as positional, we'd check args. validate_and_price signature:
    # (part, method, config, build_zone=None, quantity=1, fem_required=False)

    # Since analyze_component calls it, let's see how it calls it currently:
    # return validate_and_price(component, method=method, config=config)
    # It passes method as keyword argument.

    assert "method" in kwargs
    assert kwargs["method"] == ManufacturingMethod.THREE_DP

    # Currently quantity is NOT passed, so it uses default=1
    # We want it to be 50
    assert "quantity" in kwargs
    assert kwargs["quantity"] == 50
