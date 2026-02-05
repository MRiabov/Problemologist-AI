import sys
import uuid
import pytest
from unittest.mock import MagicMock, patch
from pathlib import Path

# Ensure src in path
project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.append(str(project_root))

from src.environment.runtime import ToolRuntime
from src.environment.persistence import DatabaseManager, ValidationRecord
from src.workbenches.models import ValidationReport, CostBreakdown, ValidationViolation

@pytest.fixture
def db_manager(tmp_path):
    db_file = tmp_path / "test_runtime.db"
    db_url = f"sqlite:///{db_file}"
    manager = DatabaseManager(db_url)
    manager.create_tables()
    return manager

@pytest.fixture
def runtime(tmp_path, db_manager):
    workspace = tmp_path / "workspace"

    # Mock PodmanSandbox to avoid needing podman installed
    with patch("src.environment.runtime.PodmanSandbox") as MockSandbox:
        mock_sandbox_instance = MockSandbox.return_value
        mock_sandbox_instance.start_session.return_value = True

        runtime = ToolRuntime(workspace_dir=str(workspace), db=db_manager)
        runtime.start_session()
        # Need to set episode for persistence to work
        runtime.episode = db_manager.create_episode("test_prob")
        return runtime

def test_persist_validation_report_analyze(runtime, db_manager):
    # Mock analyze_design to return a report
    report = ValidationReport(
        status="pass",
        manufacturability_score=0.9,
        violations=[],
        cost_analysis=CostBreakdown(
            process="cnc",
            total_cost=100.0,
            unit_cost=10.0,
            material_cost_per_unit=5.0,
            setup_cost=50.0,
            is_reused=False,
            details={"op": 1},
            pricing_explanation="test"
        ),
        stl_path="test.stl"
    )

    # Mock _run_tool to test persistence logic in dispatch
    runtime._run_tool = MagicMock(return_value=report)

    output = runtime.dispatch("analyze_design", {})

    # Check DB
    with db_manager.session_scope() as session:
        records = session.query(ValidationRecord).all()
        assert len(records) == 1
        rec = records[0]
        assert rec.status == "pass"
        assert rec.manufacturability_score == 0.9

        # Check cost breakdown
        assert rec.cost_breakdown is not None
        assert rec.cost_breakdown.process == "cnc"
        assert rec.cost_breakdown.total_cost == 100.0

def test_persist_validation_report_verify(runtime, db_manager):
    # Mock verify_solution to return dict with validation_report
    report = ValidationReport(
        status="fail",
        manufacturability_score=0.5,
        violations=[ValidationViolation(description="Bad geometry", severity="error")],
        cost_analysis=CostBreakdown(
            process="print_3d",
            total_cost=20.0,
            unit_cost=20.0,
            material_cost_per_unit=5.0,
            setup_cost=0.0,
            is_reused=False,
            details={},
            pricing_explanation=""
        ),
        stl_path=None
    )

    result = {
        "status": "fail",
        "validation_report": report.model_dump()
    }

    runtime._run_tool = MagicMock(return_value=result)

    runtime.dispatch("verify_solution", {"control_path": "foo"})

    with db_manager.session_scope() as session:
        records = session.query(ValidationRecord).all()
        assert len(records) == 1
        rec = records[0]
        assert rec.status == "fail"
        assert rec.cost_breakdown.process == "print_3d"
        # Check violations
        assert len(rec.violations_json) == 1
        assert rec.violations_json[0]["description"] == "Bad geometry"
