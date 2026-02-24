from unittest.mock import AsyncMock, patch

import pytest

from controller.agent.nodes.coder import coder_node
from controller.agent.nodes.electronics_engineer import electronics_engineer_node
from controller.agent.state import AgentState


@pytest.mark.asyncio
async def test_electromechanical_handoff():
    """Verify that Coder ignores electronics tasks and ElectronicsEngineer picks them up."""

    # 1. Setup state with a mix of tasks
    todo = """
- [ ] Create mechanical bracket
- [ ] Design circuit and route wires
"""
    state = AgentState(
        task="Build a powered bracket",
        todo=todo,
        plan="Plan details",
        journal="Starting...",
    )

    # 2. Mock Coder Node
    # Coder should pick 'Create mechanical bracket'
    with patch("controller.agent.nodes.coder.CoderNode") as MockCoder:
        mock_coder_instance = MockCoder.return_value

        # Simulating successful mechanical implementation
        updated_todo_mech = todo.replace(
            "- [ ] Create mechanical bracket", "- [x] Create mechanical bracket"
        )
        mock_coder_instance.side_effect = AsyncMock(
            return_value=state.model_copy(
                update={
                    "todo": updated_todo_mech,
                    "journal": state.journal + "\n[Coder] Finished bracket",
                }
            )
        )

        state_after_coder = await coder_node(state)
        assert "- [x] Create mechanical bracket" in state_after_coder.todo
        assert "- [ ] Design circuit and route wires" in state_after_coder.todo

    # 3. Mock Electronics Engineer Node
    # Electronics Engineer should pick 'Design circuit and route wires'
    with patch(
        "controller.agent.nodes.electronics_engineer.ElectronicsEngineerNode"
    ) as MockElec:
        mock_elec_instance = MockElec.return_value

        # Simulating successful electronics implementation
        updated_todo_elec = updated_todo_mech.replace(
            "- [ ] Design circuit and route wires",
            "- [x] Design circuit and route wires",
        )
        mock_elec_instance.side_effect = AsyncMock(
            return_value=state_after_coder.model_copy(
                update={
                    "todo": updated_todo_elec,
                    "journal": state_after_coder.journal + "\n[Elec] Finished circuit",
                }
            )
        )

        state_after_elec = await electronics_engineer_node(state_after_coder)
        assert "- [x] Design circuit and route wires" in state_after_elec.todo
        assert "[Elec] Finished circuit" in state_after_elec.journal


@pytest.mark.asyncio
async def test_electronics_engineer_passthrough():
    """Verify that ElectronicsEngineer passes through if no electronics tasks exist."""
    todo = "- [ ] Create mechanical bracket"
    state = AgentState(task="test", todo=todo)

    # It should return state unchanged
    result = await electronics_engineer_node(state)
    assert result == state
