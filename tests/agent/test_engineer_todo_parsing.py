import pytest
from unittest.mock import MagicMock, patch
from controller.agent.nodes.engineer import EngineerNode

@pytest.fixture
def node():
    with patch("controller.agent.nodes.engineer.ChatOpenAI") as mock_llm, \
         patch("controller.agent.nodes.engineer.WorkerClient") as mock_worker, \
         patch("controller.agent.nodes.engineer.RemoteFilesystemMiddleware") as mock_fs:
        yield EngineerNode(worker_url="http://mock", session_id="test")

def test_get_next_step(node):
    # Simple case
    todo = "- [ ] Step 1"
    assert node._get_next_step(todo) == "Step 1"

    # With extra spaces
    todo = "- [ ]   Step 1   "
    assert node._get_next_step(todo) == "Step 1"

    # Multiple steps
    todo = "- [x] Done\n- [ ] Next"
    assert node._get_next_step(todo) == "Next"

    # No steps
    todo = "- [x] Done"
    assert node._get_next_step(todo) is None

def test_todo_update_whitespace_handling(node):
    """Test that TODO update handles extra whitespace correctly."""
    # Case with extra spaces
    todo = "- [ ]  Step 1"

    current_step = node._get_next_step(todo)
    assert current_step == "Step 1"

    # Use the new helper method to mark the step as done
    new_todo = node._mark_step_done(todo, current_step)

    # This assertion should PASS with the new implementation
    assert "- [x]" in new_todo, "Failed to mark TODO as done due to whitespace mismatch"
    # Verify the step text is preserved
    assert f"- [x] {current_step}" in new_todo
