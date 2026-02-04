from unittest.mock import patch

from src.compiler.models import CostBreakdown, ValidationReport, ValidationViolation
from src.environment.runtime import ToolRuntime


def test_agent_exposure(tmp_path):
    # Setup environment with a temporary workspace
    workspace_dir = tmp_path / "workspace"

    # Mock PodmanSandbox and Evaluator interactions
    with (
        patch("src.environment.runtime.PodmanSandbox") as mock_sandbox,
        patch(
            "src.environment.evaluator.Evaluator.validate_and_export"
        ) as mock_validate,
    ):

        # Configure mock sandbox
        mock_sandbox_instance = mock_sandbox.return_value
        mock_sandbox_instance.start_session.return_value = True

        # Create real report object
        report = ValidationReport(
            status="fail",
            manufacturability_score=0.5,
            violations=[
                ValidationViolation(description="Thickness too small")
            ],
            cost_analysis=CostBreakdown(
                process="cnc",
                total_cost=100.0,
                unit_cost=10.0,
                material_cost_per_unit=5.0,
                setup_cost=50.0
            ),
            stl_path="design.stl"
        )

        mock_validate.return_value = report

        rt = ToolRuntime(workspace_dir=str(workspace_dir))

        # 1. Start session
        rt.start_session("test_session")

        # 2. Write a script
        design_content = "from build123d import Box\np = Box(10, 10, 10)"
        rt.dispatch("write_file", {"content": design_content, "path": "design.py"})

        # 3. Call manufacturability check
        output = rt.dispatch(
            "check_manufacturability",
            {"design_file": "design.py", "process": "cnc", "quantity": 10}
        )

        print(f"DEBUG Output: {output}")

        # 4. Assert tool output
        assert '"status": "fail"' in output
        assert "Thickness too small" in output
        assert '"process": "cnc"' in output
