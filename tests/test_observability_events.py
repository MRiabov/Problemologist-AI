from shared.observability.schemas import (
    CostWeightDeltaEvent,
    LibraryUsageEvent,
    ObservabilityEventType,
    ReviewDecisionEvent,
    SimulationFailureReason,
    SimulationInstabilityEvent,
    SubmissionValidationEvent,
)


def test_simulation_instability_event():
    event = SimulationInstabilityEvent(
        instability_type="nan",
        part_ids=["part1", "part2"],
        value=float("nan"),
        message="NaN detected in simulation",
    )
    assert event.event_type == ObservabilityEventType.SIMULATION_INSTABILITY
    data = event.model_dump(mode="json")
    assert data["instability_type"] == "nan"
    assert data["part_ids"] == ["part1", "part2"]


def test_submission_validation_event():
    event = SubmissionValidationEvent(
        artifacts_present=["plan.md", "todo.md"],
        verification_passed=True,
        reasoning_trace_quality=0.95,
        errors=[],
    )
    assert event.event_type == ObservabilityEventType.SUBMISSION_VALIDATION
    data = event.model_dump(mode="json")
    assert data["verification_passed"] is True
    assert data["reasoning_trace_quality"] == 0.95


def test_cost_weight_delta_event():
    event = CostWeightDeltaEvent(
        best_simulated_cost=100.0,
        best_simulated_weight_g=500.0,
        final_cost=120.0,
        final_weight_g=550.0,
        is_worse=True,
    )
    assert event.event_type == ObservabilityEventType.COST_WEIGHT_DELTA
    data = event.model_dump(mode="json")
    assert data["is_worse"] is True
    assert data["final_cost"] == 120.0


def test_library_usage_event():
    event = LibraryUsageEvent(
        module_name="fasteners",
        usage_type="reused",
        path="skills/fasteners/M3_screw.py",
    )
    assert event.event_type == ObservabilityEventType.LIBRARY_USAGE
    data = event.model_dump(mode="json")
    assert data["module_name"] == "fasteners"
    assert data["usage_type"] == "reused"


def test_review_decision_event():
    event = ReviewDecisionEvent(
        decision="approve", reason="Looks good", evidence_stats={"simulations_run": 5}
    )
    assert event.event_type == ObservabilityEventType.REVIEW_DECISION
    data = event.model_dump(mode="json")
    assert data["decision"] == "approve"
    assert data["evidence_stats"]["simulations_run"] == 5


def test_simulation_failure_reason_enum():
    assert SimulationFailureReason.STABILITY_ISSUE == "stability_issue"
