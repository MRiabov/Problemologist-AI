import hashlib
import json
from pathlib import Path
from typing import Any

from src.compiler.models import CostBreakdown, ValidationReport, ValidationViolation
from src.environment.sandbox_utils import run_sandboxed_script


class Evaluator:
    """
    Handles evaluation of design scripts within the sandbox environment.
    Consolidates logic for runner script generation, execution, and result parsing.
    """

    def __init__(self, sandbox: Any, workspace_dir: str):
        self.sandbox = sandbox
        self.workspace_dir = str(Path(workspace_dir).resolve())

    def _load_script_asset(self, script_name: str) -> str:
        """Loads a helper script from the assets directory."""
        # Assume assets/scripts is relative to src/assets/scripts
        # We need to find the project root or use relative path from this file

        # This file is in src/environment/evaluator.py
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

        runner_filename = "runner_preview.py"
        config_filename = "preview_config.json"

        # 1. Prepare Config
        config = {"filename": filename, "output_path": "preview_result.json"}

        # 2. Write Config and Runner to workspace (via sandbox helper or file write)
        # Assuming we can write files via sandbox helper or tool?
        # Actually run_sandboxed_script takes script_content.
        # But we want to run the script file we loaded.

        runner_content = self._load_script_asset("preview_runner.py")

        # We need to write the config file first.
        # Since run_sandboxed_script only runs ONE file, we might need a wrapper or
        # use a tool to write the config first.
        # However, for simplicity/compatibility, we can inject the config creation at the top
        # of the runner script if we really have to, OR we can depend on the agent writing it?
        # No, the evaluator should be self-contained.

        # HACK: Prepend the config writing to the runner content?
        # "with open('config.json', 'w') as f: json.dump(..., f)"
        # Or just pass arguments via command line?
        # run_sandboxed_script runs `python <file>`. We can't easily pass args unless supported.
        # Let's check `run_sandboxed_script` signature/impl. It likely just runs `python script.py`.

        # Alternative: We wrap the runner.
        # We'll use a tiny bootstrapper string that writes the config and then the real runner logic.

        bootstrap_script = f'''
import json
import sys

config = {json.dumps(config)}
with open("{config_filename}", "w") as f:
    json.dump(config, f)

# Embed the actual runner code here to ensure it runs
{runner_content}

# Now we need to make sure the runner's main() is called with the config arg
if __name__ == "__main__":
    sys.argv = ["preview_runner.py", "{config_filename}"]
    main()
'''
        # Note: The imported runner script has `if __name__ == "__main__": main()` at the bottom.
        # We need to be careful not to trigger it twice or with wrong args.
        # If we paste the content, the `if __name__` block will execute.
        # Since we control the pasted content, we can strip the last lines or just override sys.argv BEFORE the paste.
        # Better: The pasted content defines main(). We just call it.

        # Let's simplify: We'll overwrite the `if __name__ == "__main__":` block in the loaded content
        # or just rely on the fact that if we use `run_sandboxed_script`, it saves the content to `runner_filename`.
        # And runs `python runner_filename`.
        # So we can just bake the arg into the script or use a fixed config filename.

        # Let's bake the config into a file using a preamble.

        runner_with_config = (
            f"""
import json
import sys

# Write config
with open("{config_filename}", "w") as f:
    f.write({repr(json.dumps(config))})

# Set args so main() picks it up
sys.argv = ["runner.py", "{config_filename}"]

"""
            + runner_content
        )

        res = run_sandboxed_script(
            sandbox=self.sandbox,
            script_content=runner_with_config,
            result_file_name="preview_result.json",
            runner_file_name=runner_filename,
            session_id=session_id,
        )

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
        config_filename = "validation_config.json"
        result_file = "validation_result.json"

        # 1. Prepare Config
        config = {
            "design_file": design_file,
            "process": process,
            "quantity": quantity,
            "export_stl": export_stl,
            "output_path": result_file,
        }

        # 2. Load runner code
        runner_content = self._load_script_asset("validation_runner.py")

        # 3. Inject config writer preamble
        runner_with_config = (
            f"""
import json
import sys

# Write config
with open("{config_filename}", "w") as f:
    f.write({repr(json.dumps(config))})

# Set args
sys.argv = ["runner.py", "{config_filename}"]

"""
            + runner_content
        )

        res = run_sandboxed_script(
            sandbox=self.sandbox,
            script_content=runner_with_config,
            result_file_name=result_file,
            runner_file_name=runner_filename,
            session_id=session_id,
        )

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
        # Note: ValidationReport expects objects, but pydantic can validate from dict if configured.
        # But we are constructing it manually here to be safe and explicit.

        cost_info = res.get("cost_analysis", {})
        cost_breakdown = CostBreakdown(
            process=process,
            total_cost=cost_info.get("total_cost", 0.0),
            unit_cost=cost_info.get("unit_cost", 0.0),
            material_cost_per_unit=0.0,  # This is lost in aggregation unless we check details
            setup_cost=0.0,
            details={"parts": res.get("parts", [])},
        )

        violations = [
            ValidationViolation(description=v["description"])
            for v in res.get("violations", [])
        ]

        return ValidationReport(
            status=res.get("status", "fail"),
            manufacturability_score=res.get("manufacturability_score", 0.0),
            violations=violations,
            cost_analysis=cost_breakdown,
            parts=res.get("parts", []),
            stl_path=res.get("stl_path"),
            error=res.get("error"),
        )
