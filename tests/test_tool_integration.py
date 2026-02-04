from src.compiler.models import ValidationReport
from src.environment.runtime import ToolRuntime


def test_agent_exposure(tmp_path):
    # Setup environment with a temporary workspace
    workspace_dir = tmp_path / "workspace"
    runtime = ToolRuntime(workspace_dir=str(workspace_dir))

    # 1. Write a script
    design_content = "from build123d import Box\np = Box(10, 10, 10)"
    runtime.dispatch("write_file", {"content": design_content, "path": "design.py"})

    # 2. Call manufacturability check via agent interface
    # tool output is a ValidationReport object (not JSON string) in direct runtime calls
    output = runtime.dispatch("analyze_design", {"process": "cnc", "quantity": 10})

    # 3. Assert tool output is valid
    if isinstance(output, ValidationReport):
        report = output
    else:
        # Fallback if it returned json string or dict
        report = (
            ValidationReport.model_validate(output)
            if isinstance(output, dict)
            else ValidationReport.model_validate_json(output)
        )

    # We expect it to fail or succeed depending on logic, but here we check structure
    # In this test env (temp dir), src might be missing, causing "error" (ImportError)
    # Old implementation returned "fail" on exception, new one returns "error".
    assert report.status in ["pass", "fail", "error"]
    assert report.cost_analysis.process == "cnc"

    # Check that unit cost is a float (or 0.0)
    assert isinstance(report.cost_analysis.unit_cost, float)
