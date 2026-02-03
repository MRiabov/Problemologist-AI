import hashlib

from pathlib import Path
from typing import Any

from src.environment.sandbox_utils import run_sandboxed_script


class Evaluator:
    """
    Handles evaluation of design scripts within the sandbox environment.
    Consolidates logic for runner script generation, execution, and result parsing.
    """

    def __init__(self, sandbox: Any, workspace_dir: str):
        self.sandbox = sandbox
        self.workspace_dir = str(Path(workspace_dir).resolve())

    def _get_runner_script_prefix(self) -> str:
        return """
import sys
import json
import hashlib
from pathlib import Path
import build123d as bd

sys.path.append("/workspace")

def get_export_obj(locs):
    # Prefer 'result' if it exists and is a valid shape or list of shapes
    if "result" in locs:
        val = locs["result"]
        if isinstance(val, (bd.Compound, bd.Solid, bd.Shape)):
            return val
        if isinstance(val, list) and all(isinstance(x, (bd.Compound, bd.Solid, bd.Shape)) for x in val):
            return bd.Compound(val)

    for val in locs.values():
        if isinstance(val, (bd.Compound, bd.Solid, bd.Shape)):
            return val
        elif hasattr(val, "part") and isinstance(val.part, bd.Shape):
            return val.part
        elif hasattr(val, "sketch") and isinstance(val.sketch, bd.Shape):
            return val.sketch
        elif hasattr(val, "line") and isinstance(val.line, bd.Shape):
            return val.line
    return None

def parse_label(label, default_q, default_p):
    data = {"quantity": default_q, "process": default_p}
    if not label: return data
    for p in str(label).split("|"):
        if ":" in p:
            k, v = p.split(":", 1)
            if k == "quantity":
                try: data["quantity"] = int(v)
                except: pass
            elif k == "process": data["process"] = v
    return data
"""

    def preview_design(
        self, filename: str = "design.py", session_id: str | None = None
    ) -> str:
        """Executes the script and renders result to SVG."""
        filename = Path(filename).name
        script_path = Path(self.workspace_dir) / filename

        if not script_path.exists():
            return f"Error: File {filename} does not exist."

        runner_filename = f"runner_{filename}"
        result_file = "preview_result.json"

        runner_script = (
            self._get_runner_script_prefix()
            + f"""
from build123d import ExportSVG, Drawing, Unit

result = {{
    "status": "error",
    "preview_file": None,
    "error": None
}}

try:
    locs = {{}}
    with Path("/workspace/{filename}").open("r", encoding="utf-8") as f:
        code = f.read()
    exec(code, globals(), locs)

    export_obj = get_export_obj(locs)

    if export_obj:
        svg_filename = f"{{Path('{filename}').stem}}.svg"
        svg_path = Path("/workspace") / svg_filename

        try:
            drawing = Drawing(export_obj)
            exporter = ExportSVG(unit=Unit.MM)
            exporter.add_layer("visible", line_color=(0,0,0), line_weight=0.2)
            exporter.add_shape(drawing.visible_lines, layer="visible")
            exporter.write(str(svg_path))
            result["status"] = "success"
            result["preview_file"] = svg_filename
        except Exception as e:
            result["error"] = str(e)
    else:
        result["error"] = "No 3D object found"

except Exception as e:
    result["error"] = str(e)

with open("/workspace/{result_file}", "w") as f:
    json.dump(result, f)
"""
        )

        res = run_sandboxed_script(
            sandbox=self.sandbox,
            script_content=runner_script,
            result_file_name=result_file,
            runner_file_name=runner_filename,
            session_id=session_id,
        )

        # Check for system/sandbox errors
        if res.get("status") == "error" and "error_type" in res:
            # This is a sandbox error (timeout, crash, etc)
            return f"Error generating preview: {res.get('message')}"

        # Check for script errors (from the JSON result)
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
    ) -> dict[str, Any]:
        """Validates design for manufacturability and optionally exports STL."""
        design_file = Path(design_file).name
        script_path = Path(self.workspace_dir) / design_file

        if not script_path.exists():
            return {"error": f"File {design_file} does not exist."}

        runner_filename = (
            f"val_runner_{hashlib.md5(design_file.encode()).hexdigest()[:8]}.py"
        )
        result_file = "validation_result.json"
        stl_filename = "design.stl"

        runner_script = (
            self._get_runner_script_prefix()
            + f"""
from src.workbenches.cnc import CNCWorkbench
from src.workbenches.injection_molding import InjectionMoldingWorkbench
from src.workbenches.print_3d import Print3DWorkbench
from src.compiler import geometry

result = {{
    "status": "fail",
    "manufacturability_score": 0.0,
    "violations": [],
    "parts": [],
    "cost_analysis": {{"total_cost": 0.0, "unit_cost": 0.0, "target_quantity": {quantity}}},
    "stl_path": None,
    "error": None
}}

try:
    locs = {{}}
    with Path("/workspace/{design_file}").open("r", encoding="utf-8") as f:
        code = f.read()
    exec(code, globals(), locs)

    export_obj = get_export_obj(locs)
    if not export_obj:
        raise ValueError("No 3D object found")

    all_solids = list(export_obj.solids()) if isinstance(export_obj, bd.Compound) else [export_obj]

    workbenches = {{
        "cnc": CNCWorkbench(), "injection_molding": InjectionMoldingWorkbench(), "print_3d": Print3DWorkbench()
    }}

    total_cost, all_violations, part_reports, reuse_ctx = 0.0, [], [], {{}}

    for i, solid in enumerate(all_solids):
        ldata = parse_label(getattr(solid, "label", ""), {quantity}, "{process}")
        wb = workbenches.get(ldata["process"], workbenches.get("{process}", workbenches["print_3d"]))

        violations = wb.validate(solid)
        all_violations.extend(violations)
        cost_res = wb.calculate_cost(solid, ldata["quantity"], context=reuse_ctx)
        
        cost = 0.0
        breakdown = None

        if isinstance(cost_res, dict):
             cost = cost_res.get("total_cost", 0.0)
             breakdown = cost_res.get("breakdown")
        else:
             cost = float(cost_res)

        total_cost += cost

        part_reports.append({{
            "part_index": i, "process": ldata["process"], "quantity": ldata["quantity"],
            "cost": cost, "breakdown": breakdown, "violations": [str(v) for v in violations]
        }})

    result["status"] = "pass" if not all_violations else "fail"
    result["manufacturability_score"] = max(0.0, 1.0 - len(all_violations)*0.1)
    result["violations"] = [{{"description": str(v)}} for v in all_violations]
    result["parts"], result["cost_analysis"]["total_cost"] = part_reports, total_cost
    result["cost_analysis"]["unit_cost"] = total_cost / {quantity} if {quantity} > 0 else 0.0

    if {export_stl} and not all_violations:
        geometry.export_mesh(export_obj, "/workspace/{stl_filename}")
        result["stl_path"] = "{stl_filename}"

except Exception as e:
    result["error"] = str(e)

with open("/workspace/{result_file}", "w") as f:
    json.dump(result, f)
"""
        )

        res = run_sandboxed_script(
            sandbox=self.sandbox,
            script_content=runner_script,
            result_file_name=result_file,
            runner_file_name=runner_filename,
            session_id=session_id,
        )

        if res.get("status") == "error" and "error_type" in res:
            return {"error": f"Exec failed: {res.get('message')}"}

        return res
