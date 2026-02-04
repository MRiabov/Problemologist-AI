import hashlib
import json
from pathlib import Path
from typing import Any

from src.environment.config import CONTAINER_WORKSPACE, RUNNER_JOB_CONFIG
from src.workbenches.models import CostBreakdown, ValidationReport, ValidationViolation
from src.environment.sandbox_utils import run_sandboxed_script


class DesignExecutor:
    """
    Handles execution and validation of design scripts within the sandbox environment.
    Consolidates logic for runner script generation, execution, and result parsing.
    """

    def __init__(self, sandbox: Any, workspace_dir: str):
        self.sandbox = sandbox
        self.workspace_dir = str(Path(workspace_dir).resolve())

    def _load_script_asset(self, script_name: str) -> str:
        """Loads a helper script from the assets directory."""
        # Assume assets/scripts is relative to src/assets/scripts
        # We need to find the project root or use relative path from this file

        # This file is in src/environment/design_executor.py
        # scripts are in src/assets/scripts/
        base_path = Path(__file__).parent.parent / "assets" / "scripts"
        script_path = base_path / script_name

        if not script_path.exists():
            raise FileNotFoundError(
                f"Runner script {script_name} not found at {script_path}"
            )

        return script_path.read_text(encoding="utf-8")

    def preview_design(
        self, filename: str = "design.py", session_id: str | None = None
    ) -> str:
        """Executes the script and renders result to SVG."""
        filename = Path(filename).name
        script_path = Path(self.workspace_dir) / filename

        if not script_path.exists():
            return f"Error: File {filename} does not exist."

        # 1. Prepare Config
        config = {"filename": filename, "output_path": "preview_result.json"}

        runner_filename = "preview_runner_exec.py"
        runner_content = self._load_script_asset("preview_runner.py")

        # 2. Write Config and Runner to workspace (via sandbox helper or file write)
        # Assuming we can write files via sandbox helper or tool?
        # Actually run_sandboxed_script takes script_content.
        # But we want to run the script file we loaded.

        # 2. Write Config to workspace
        config_path = Path(self.workspace_dir) / RUNNER_JOB_CONFIG
        config_path.write_text(json.dumps(config), encoding="utf-8")

        res = run_sandboxed_script(
            sandbox=self.sandbox,
            script_content=runner_content,
            result_file_name="preview_result.json",
            runner_file_name=runner_filename,
            session_id=session_id,
        )

        # Cleanup config
        if config_path.exists():
            config_path.unlink()

        # Check for system/sandbox errors
        if res.get("status") == "error" and "error_type" in res:
            return f"Error generating preview: {res.get('message')}"

        # Check for script errors
        if res.get("status") == "success":
            return f"Preview generated: {res['preview_file']}"

        return f"Error generating preview: {res.get('error', 'Unknown error')}"

    def validate_and_export(
        self,
        design_file: str = "design.py",
        process: str = "cnc",
        quantity: int = 1,
        export_stl: bool = False,
        session_id: str | None = None,
    ) -> ValidationReport:
        """Validates design for manufacturability and optionally exports STL."""
        design_file = Path(design_file).name
        script_path = Path(self.workspace_dir) / design_file

        if not script_path.exists():
            return ValidationReport(
                status="fail",
                manufacturability_score=0.0,
                violations=[],
                cost_analysis=CostBreakdown(
                    process=process,
                    total_cost=0.0,
                    unit_cost=0.0,
                    material_cost_per_unit=0.0,
                ),
                error=f"File {design_file} does not exist.",
            )

        runner_filename = (
            f"val_runner_{hashlib.md5(design_file.encode()).hexdigest()[:8]}.py"
        )
        result_file = "validation_result.json"

        # 1. Prepare Config
        config = {
            "design_file": design_file,
            "process": process,
            "quantity": quantity,
            "export_stl": export_stl,
            "output_path": result_file,
        }

        runner_content = self._load_script_asset("validation_runner.py")

        # 3. Write Config to workspace
        config_path = Path(self.workspace_dir) / RUNNER_JOB_CONFIG
        config_path.write_text(json.dumps(config), encoding="utf-8")

        res = run_sandboxed_script(
            sandbox=self.sandbox,
            script_content=runner_content,
            result_file_name=result_file,
            runner_file_name=runner_filename,
            session_id=session_id,
        )

        # Cleanup config
        if config_path.exists():
            config_path.unlink()

        if res.get("status") == "error" and "error_type" in res:
            return ValidationReport(
                status="fail",
                manufacturability_score=0.0,
                violations=[],
                cost_analysis=CostBreakdown(
                    process=process,
                    total_cost=0.0,
                    unit_cost=0.0,
                    material_cost_per_unit=0.0,
                ),
                error=f"Exec failed: {res.get('message')}",
            )

        # Reconstruct models from JSON response
        from src.workbenches.models import ValidationStatus

        status_raw = res.get("status", "fail")
        if status_raw in ["pass", "success", "ok"]:
            status = ValidationStatus.PASSED
        else:
            status = ValidationStatus.FAILED

        cost_info = res.get("cost_analysis", {})
        cost_breakdown = CostBreakdown(
            process=process,
            total_cost=cost_info.get("total_cost", 0.0),
            unit_cost=cost_info.get("unit_cost", 0.0),
            material_cost_per_unit=0.0,
            setup_cost=0.0,
            details={"parts": res.get("parts", [])},
        )

        violations = [
            ValidationViolation(description=v["description"])
            for v in res.get("violations", [])
        ]

        return ValidationReport(
            status=status,
            manufacturability_score=res.get("manufacturability_score", 0.0),
            violations=violations,
            cost_analysis=cost_breakdown,
            parts=res.get("parts", []),
            stl_path=res.get("stl_path"),
            error=res.get("error"),
        )
