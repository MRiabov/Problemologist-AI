import pytest
from controller.agent.mock_llm import MockDSPyLM


def test_detect_node_key_explicit():
    # Use benchmark_planner which should map to 'planner' lookup key
    mock_lm = MockDSPyLM(session_id="benchmark", node_type="benchmark_planner")

    # Prompt MUST contain 'json' to trigger JSON response in MockDSPyLM
    prompt = "Some text with json output fields"

    # Mocking self.scenarios to be simple
    mock_lm.scenarios = {"benchmark": {"planner": {"thought": "Explicit Success"}}}

    responses = mock_lm(prompt=prompt)
    import json

    resp_data = json.loads(responses[0])
    assert resp_data["thought"] == "Explicit Success"


def test_detect_node_key_fallback_still_works():
    mock_lm = MockDSPyLM(session_id="default", node_type=None)

    # Header based detection
    prompt = "You are the Sidecar Learner."
    node_key = mock_lm._detect_node_key(prompt)
    assert node_key == "skill_learner"

    prompt_2 = "You are an expert designer of spatial and geometric puzzles."
    node_key_2 = mock_lm._detect_node_key(prompt_2)
    assert node_key_2 == "planner"


def test_normalization_logic():
    mock_lm = MockDSPyLM(session_id="benchmark")

    mock_lm.scenarios = {
        "benchmark": {
            "planner": {"thought": "P"},
            "coder": {"thought": "C"},
            "reviewer": {"thought": "R"},
        }
    }

    prompt = "json output fields"

    mock_lm.node_type = "benchmark_planner"
    assert json_thought(mock_lm(prompt)) == "P"

    mock_lm.node_type = "benchmark_coder"
    assert json_thought(mock_lm(prompt)) == "C"

    mock_lm.node_type = "benchmark_reviewer"
    assert json_thought(mock_lm(prompt)) == "R"

    mock_lm.node_type = "plan_reviewer"
    assert json_thought(mock_lm(prompt)) == "R"

    mock_lm.node_type = "execution_reviewer"
    assert json_thought(mock_lm(prompt)) == "R"


def json_thought(responses):
    import json

    return json.loads(responses[0])["thought"]
