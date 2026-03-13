import pytest

from controller.agent.mock_llm import MockDSPyLM
from shared.enums import AgentName


def test_detect_node_key_explicit():
    mock_lm = MockDSPyLM(session_id="benchmark", node_type=AgentName.BENCHMARK_PLANNER)

    # Prompt MUST contain 'json' to trigger JSON response in MockDSPyLM
    prompt = "Some text with json output fields"

    # Mocking self.scenarios to be simple
    mock_lm.scenarios = {
        "benchmark": {"benchmark_planner": {"thought": "Explicit Success"}}
    }

    responses = mock_lm(prompt=prompt)
    import json

    resp_data = json.loads(responses[0])
    assert resp_data["thought"] == "Explicit Success"


def test_requires_explicit_node_type():
    mock_lm = MockDSPyLM(session_id="default", node_type=None)
    mock_lm.scenarios = {"default": {}}
    with pytest.raises(ValueError, match="requires explicit node_type"):
        mock_lm(prompt="any prompt")


def test_normalization_logic():
    mock_lm = MockDSPyLM(session_id="benchmark", node_type=AgentName.BENCHMARK_PLANNER)

    mock_lm.scenarios = {
        "benchmark": {
            "benchmark_planner": {"thought": "P"},
            "benchmark_plan_reviewer": {"thought": "PR"},
            "benchmark_coder": {"thought": "C"},
            "benchmark_reviewer": {"thought": "R"},
            "engineer_plan_reviewer": {"thought": "R"},
            "engineer_execution_reviewer": {"thought": "R"},
        }
    }

    prompt = "json output fields"

    mock_lm.node_type = AgentName.BENCHMARK_PLANNER
    assert json_thought(mock_lm(prompt)) == "P"

    mock_lm.node_type = AgentName.BENCHMARK_PLAN_REVIEWER
    assert json_thought(mock_lm(prompt)) == "PR"

    mock_lm.node_type = AgentName.BENCHMARK_CODER
    assert json_thought(mock_lm(prompt)) == "C"

    mock_lm.node_type = AgentName.BENCHMARK_REVIEWER
    assert json_thought(mock_lm(prompt)) == "R"

    mock_lm.node_type = AgentName.ENGINEER_PLAN_REVIEWER
    assert json_thought(mock_lm(prompt)) == "R"

    mock_lm.node_type = AgentName.ENGINEER_EXECUTION_REVIEWER
    assert json_thought(mock_lm(prompt)) == "R"


def json_thought(responses):
    import json

    return json.loads(responses[0])["thought"]


def test_expected_llm_inputs_optional_assertions():
    mock_lm = MockDSPyLM(session_id="INT-TEST")
    node_data = {
        "expected_llm_inputs": [{"contains": ["alpha"], "not_contains": ["forbidden"]}]
    }
    mock_lm._validate_expected_llm_inputs(
        node_key="planner",
        node_data=node_data,
        full_text="this input has alpha token",
        call_index=1,
    )

    with pytest.raises(AssertionError):
        mock_lm._validate_expected_llm_inputs(
            node_key="planner",
            node_data=node_data,
            full_text="this input has forbidden token",
            call_index=1,
        )


def test_expected_tool_calls_and_outputs_optional_assertions():
    mock_lm = MockDSPyLM(session_id="INT-TEST")
    node_data = {
        "expected_tool_calls": [
            {"name": "write_file", "args_contains": ["plan.md", "overwrite"]}
        ],
        "expected_tool_outputs": [{"contains": ["success"], "not_contains": ["error"]}],
    }
    full_text = """
    [[ ## tool_name_0 ## ]]
    write_file

    [[ ## tool_args_0 ## ]]
    {"path":"plan.md","overwrite":true}

    [[ ## observation_0 ## ]]
    success: file written
    """
    mock_lm._validate_expected_tool_calls(
        node_key="planner",
        node_data=node_data,
        full_text=full_text,
    )
    mock_lm._validate_expected_tool_outputs(
        node_key="planner",
        node_data=node_data,
        full_text=full_text,
    )

    with pytest.raises(AssertionError):
        mock_lm._validate_expected_tool_calls(
            node_key="planner",
            node_data={"expected_tool_calls": [{"name": "edit_file"}]},
            full_text=full_text,
        )
