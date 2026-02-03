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
    # tool output is a JSON string of the ValidationReport
    output_str = runtime.dispatch(
        "check_manufacturability", {"process": "cnc", "quantity": 10}
    )

    # 3. Assert tool output is valid JSON matching our Model
    report = ValidationReport.model_validate_json(output_str)

    # We expect it to fail or succeed depending on logic, but here we check structure
    # Based on previous test it was "status": "fail"
    assert report.status in ["pass", "fail"]
    assert report.cost_analysis.process == "cnc"

    # Check that unit cost is a float (or 0.0)
    assert isinstance(report.cost_analysis.unit_cost, float)
