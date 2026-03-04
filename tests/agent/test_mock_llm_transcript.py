from unittest.mock import patch

import pytest

from controller.agent.mock_llm import MockDSPyLM
from shared.enums import AgentName


@pytest.fixture
def mock_scenarios():
    return {
        "TRANSCRIPT-TEST": {
            "transcript": [
                {
                    "node": AgentName.ENGINEER_PLANNER.value,
                    "steps": [
                        {
                            "thought": "First thought",
                            "tool_name": "write_file",
                            "tool_args": {"path": "plan.md", "content": "Initial plan"},
                            "expected_observation": "success",
                        },
                        {
                            "thought": "Finished planning",
                            "finished": True,
                            "summary": "Plan is ready",
                        },
                    ],
                },
                {
                    "node": AgentName.ENGINEER_CODER.value,
                    "steps": [
                        {
                            "thought": "Coding now",
                            "finished": True,
                            "journal": "Coded everything",
                        }
                    ],
                },
            ]
        }
    }


def test_transcript_multi_node_sequence(mock_scenarios):
    # Reset transcript states before test
    MockDSPyLM._transcript_states = {}
    with patch.object(MockDSPyLM, "_load_scenarios", return_value=mock_scenarios):
        # 1. Planner Turn - Step 0
        lm = MockDSPyLM(
            session_id="TRANSCRIPT-TEST", node_type=AgentName.ENGINEER_PLANNER
        )
        # next_tool_name in prompt triggers ReAct tool selection fields
        resp = lm(prompt="I need to plan. next_tool_name")
        assert "[[ ## next_tool_name ## ]]\nwrite_file" in resp[0]
        assert (
            '[[ ## next_tool_args ## ]]\n{"path": "plan.md", "content": "Initial plan"}'
            in resp[0]
        )

        # 2. Planner Turn - Step 1 (after observation)
        # summary in prompt triggers signature fields
        resp = lm(prompt="I need to plan. Observation: success. summary")
        assert "[[ ## finished ## ]]\ntrue" in resp[0]
        assert "[[ ## summary ## ]]\nPlan is ready" in resp[0]

        # 3. Coder Turn
        lm_coder = MockDSPyLM(
            session_id="TRANSCRIPT-TEST", node_type=AgentName.ENGINEER_CODER
        )
        resp = lm_coder(prompt="I am the coder. journal")
        assert "[[ ## finished ## ]]\ntrue" in resp[0]
        assert "[[ ## journal ## ]]\nCoded everything" in resp[0]


def test_transcript_idempotency(mock_scenarios):
    MockDSPyLM._transcript_states = {}
    with patch.object(MockDSPyLM, "_load_scenarios", return_value=mock_scenarios):
        lm = MockDSPyLM(
            session_id="TRANSCRIPT-TEST", node_type=AgentName.ENGINEER_PLANNER
        )

        # Call multiple times with same prompt
        resp1 = lm(prompt="Plan please next_tool_name")
        resp2 = lm(prompt="Plan please next_tool_name")

        assert resp1 == resp2
        assert "[[ ## next_tool_name ## ]]\nwrite_file" in resp1[0]


def test_transcript_observation_validation(mock_scenarios):
    MockDSPyLM._transcript_states = {}
    with patch.object(MockDSPyLM, "_load_scenarios", return_value=mock_scenarios):
        lm = MockDSPyLM(
            session_id="TRANSCRIPT-TEST", node_type=AgentName.ENGINEER_PLANNER
        )

        # First call triggers step 0
        lm(prompt="Plan please next_tool_name")

        # Second call with WRONG observation should log error but continue for now
        lm(prompt="Observation: error occurred. summary")


def test_transcript_fallback_to_legacy(mock_scenarios):
    MockDSPyLM._transcript_states = {}
    # Add a legacy node to scenarios
    mock_scenarios["LEGACY-TEST"] = {
        AgentName.ENGINEER_PLANNER.value: {
            "thought": "Legacy thought",
            "summary": "Legacy summary",
        }
    }

    with patch.object(MockDSPyLM, "_load_scenarios", return_value=mock_scenarios):
        lm = MockDSPyLM(session_id="LEGACY-TEST", node_type=AgentName.ENGINEER_PLANNER)
        resp = lm(prompt="Legacy plan. summary")
        assert "[[ ## summary ## ]]\nLegacy summary" in resp[0]
        assert "[[ ## thought ## ]]\nLegacy thought" in resp[0]
