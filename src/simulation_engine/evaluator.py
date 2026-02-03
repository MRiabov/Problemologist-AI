"""
Evaluator Module.
Handles generation of runner scripts for design analysis and validation.
Consolidates logic previously duplicated in Runtime and CADEnv.
"""

import hashlib
from pathlib import Path

def _get_runner_script_prefix() -> str:
    return """
import sys
import json
import hashlib
from pathlib import Path
import build123d as bd

sys.path.append("/workspace")

def get_export_obj(locs):
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

def generate_preview_script(filename: str, output_json_path: str = "_preview_result.json") -> str:
    """Generates a script to preview the design and output result to JSON."""
    return (
        _get_runner_script_prefix()
        + f"""
from build123d import ExportSVG, Drawing, Unit

result = {{
    "status": "error",
    "generated_file": None,
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
            result["generated_file"] = svg_filename
        except Exception as e:
            result["error"] = str(e)
    else:
        result["error"] = "No 3D object found"
except Exception as e:
    result["error"] = str(e)

with open("{output_json_path}", "w") as f:
    json.dump(result, f)
"""
    )

def generate_validation_script(
    design_file: str,
    process: str,
    quantity: int,
    export_stl: bool,
    output_json_path: str = "_val_result.json",
    stl_filename: str = "design.stl"
) -> str:
    """Generates a script to validate the design and output result to JSON."""
    return (
        _get_runner_script_prefix()
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
        cost = wb.calculate_cost(solid, ldata["quantity"], context=reuse_ctx)
        total_cost += cost

        part_reports.append({{
            "part_index": i, "process": ldata["process"], "quantity": ldata["quantity"],
            "cost": cost, "violations": [str(v) for v in violations]
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

with open("{output_json_path}", "w") as f:
    json.dump(result, f)
"""
    )
