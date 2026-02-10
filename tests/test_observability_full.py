import pytest
from datetime import datetime, UTC
from shared.observability.schemas import (
    ComponentUsageEvent,
    ToolInvocationEvent,
    ManufacturabilityCheckEvent,
    SceneValidationEvent,
    RenderRequestEngineerEvent,
    RenderRequestBenchmarkEvent,
    SimulationRequestEvent,
    SimulationResultEvent,
    COTSSearchEvent,
    PlanSubmissionBenchmarkEvent,
    PlanSubmissionEngineerEvent,
    EscalationRequestEvent,
    EscalationDecisionEvent,
    LintFailureCodeEvent,
    LintFailureDocsEvent,
    LogicFailureEvent,
    SkillEditEvent,
    SkillReadEvent,
    LsFilesToolEvent,
    GrepToolEvent,
    ReadFileToolEvent,
    WriteFileToolEvent,
    EditFileToolEvent,
    RunCommandToolEvent,
    GitInitToolEvent,
    GitCommitToolEvent,
    SimulationInstabilityEvent,
    SubmissionValidationEvent,
    CostWeightDeltaEvent,
    LibraryUsageEvent,
    ReviewDecisionEvent,
    ObservabilityEventType,
    SimulationFailureReason,
)


def test_component_usage_event():
    event = ComponentUsageEvent(
        category="motor",
        part_number="M123",
        label="Main Motor",
        price=12.50,
        weight_g=150.0,
    )
    data = event.model_dump(mode="json")
    assert data["event_type"] == ObservabilityEventType.COMPONENT_USAGE
    assert data["price"] == 12.50


def test_tool_invocation_event():
    event = ToolInvocationEvent(tool_name="ls_files", arguments={"path": "."})
    assert event.event_type == ObservabilityEventType.TOOL_INVOCATION


def test_manufacturability_check_event():
    event = ManufacturabilityCheckEvent(
        part_id="part_1",
        method="cnc",
        result=True,
        price=10.0,
        weight_g=50.0,
        metadata={"material": "aluminum"},
    )
    assert event.event_type == ObservabilityEventType.MANUFACTURABILITY_CHECK


def test_scene_validation_event():
    event = SceneValidationEvent(result=False, errors=["Overlapping parts"])
    assert event.event_type == ObservabilityEventType.SCENE_VALIDATION


def test_render_requests():
    e1 = RenderRequestEngineerEvent(num_views=12)
    assert e1.event_type == ObservabilityEventType.RENDER_REQUEST_ENGINEER

    e2 = RenderRequestBenchmarkEvent(num_views=4)
    assert e2.event_type == ObservabilityEventType.RENDER_REQUEST_BENCHMARK


def test_simulation_events():
    req = SimulationRequestEvent(script_path="sim.py")
    assert req.event_type == ObservabilityEventType.SIMULATION_REQUEST

    res = SimulationResultEvent(
        success=False,
        failure_reason=SimulationFailureReason.TIMEOUT,
        time_elapsed_s=10.5,
        compute_time_ms=500.0,
    )
    assert res.event_type == ObservabilityEventType.SIMULATION_RESULT
    assert res.failure_reason == "timeout"


def test_cots_search_event():
    event = COTSSearchEvent(query="M3 bolts", results_count=5)
    assert event.event_type == ObservabilityEventType.COTS_SEARCH


def test_plan_submission_events():
    e1 = PlanSubmissionBenchmarkEvent(plan_path="plan.md")
    assert e1.event_type == ObservabilityEventType.PLAN_SUBMISSION_BENCHMARK

    e2 = PlanSubmissionEngineerEvent(plan_path="plan.md")
    assert e2.event_type == ObservabilityEventType.PLAN_SUBMISSION_ENGINEER


def test_escalation_events():
    req = EscalationRequestEvent(reason="Too heavy", current_weight=2.0)
    assert req.event_type == ObservabilityEventType.ESCALATION_REQUEST

    dec = EscalationDecisionEvent(decision="approved", comments=["Go ahead"])
    assert dec.event_type == ObservabilityEventType.ESCALATION_DECISION


def test_lint_failure_events():
    code = LintFailureCodeEvent(file_path="main.py", errors=["E501 line too long"])
    assert code.event_type == ObservabilityEventType.LINT_FAILURE_CODE

    docs = LintFailureDocsEvent(file_path="doc.md", errors=["Missing header"])
    assert docs.event_type == ObservabilityEventType.LINT_FAILURE_DOCS


def test_logic_failure_event():
    event = LogicFailureEvent(
        file_path="logic.py", constraint_name="max_load", error_message="Exceeded 5kg"
    )
    assert event.event_type == ObservabilityEventType.LOGIC_FAILURE


def test_skill_events():
    edit = SkillEditEvent(skill_name="drilling", action="update", lines_changed=5)
    assert edit.event_type == ObservabilityEventType.SKILL_EDIT

    read = SkillReadEvent(skill_path="skills/drilling.py", skill_name="drilling")
    assert read.event_type == ObservabilityEventType.SKILL_READ


def test_tool_specific_events():
    ls = LsFilesToolEvent(path=".")
    assert ls.event_type == ObservabilityEventType.TOOL_LS_FILES

    grep = GrepToolEvent(pattern="TODO", path=".")
    assert grep.event_type == ObservabilityEventType.TOOL_GREP

    read = ReadFileToolEvent(path="file.txt")
    assert read.event_type == ObservabilityEventType.TOOL_READ_FILE

    write = WriteFileToolEvent(path="file.txt", content_snippet="content")
    assert write.event_type == ObservabilityEventType.TOOL_WRITE_FILE

    edit = EditFileToolEvent(path="file.txt", num_edits=1)
    assert edit.event_type == ObservabilityEventType.TOOL_EDIT_FILE

    run = RunCommandToolEvent(command="ls -la")
    assert run.event_type == ObservabilityEventType.TOOL_RUN_COMMAND

    git_init = GitInitToolEvent()
    assert git_init.event_type == ObservabilityEventType.TOOL_GIT_INIT

    git_commit = GitCommitToolEvent(message="feat: add file")
    assert git_commit.event_type == ObservabilityEventType.TOOL_GIT_COMMIT


def test_simulation_instability():
    event = SimulationInstabilityEvent(
        instability_type="nan",
        part_ids=["p1"],
        value=float("nan"),
        message="Energy explode",
    )
    assert event.event_type == ObservabilityEventType.SIMULATION_INSTABILITY


def test_submission_validation():
    event = SubmissionValidationEvent(
        artifacts_present=["plan.md"],
        verification_passed=True,
        reasoning_trace_quality=0.8,
    )
    assert event.event_type == ObservabilityEventType.SUBMISSION_VALIDATION


def test_cost_weight_delta():
    event = CostWeightDeltaEvent(
        best_simulated_cost=10.0,
        best_simulated_weight_g=100.0,
        final_cost=12.0,
        final_weight_g=105.0,
        is_worse=True,
    )
    assert event.event_type == ObservabilityEventType.COST_WEIGHT_DELTA


def test_library_usage():
    event = LibraryUsageEvent(
        module_name="generic_wheel", usage_type="reused", path="libs/wheel.py"
    )
    assert event.event_type == ObservabilityEventType.LIBRARY_USAGE


def test_review_decision():
    event = ReviewDecisionEvent(
        decision="reject_code", reason="Bugs found", evidence_stats={"bugs": 3}
    )
    assert event.event_type == ObservabilityEventType.REVIEW_DECISION
