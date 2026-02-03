import sys
import json
import dataclasses
import traceback
from pathlib import Path
from typing import Any

import build123d as bd

# Import project modules (assumes src is available in pythonpath or mounted)
try:
    from src.workbenches.cnc import CNCWorkbench
    from src.workbenches.injection_molding import InjectionMoldingWorkbench
    from src.workbenches.print_3d import Print3DWorkbench
    from src.compiler import geometry
    from src.compiler.models import ValidationReport, CostBreakdown, ValidationViolation
except ImportError as e:
    print(f"Critical Import Error: {e}")
    # We might need to adjust sys.path if src is not in root
    sys.path.append("/workspace")


# Re-define helper since we don't want to import it from evaluator to avoid circular deps with logic there
def get_export_obj(locs: dict[str, Any]) -> Any:
    # Prefer 'result' if it exists and is a valid shape or list of shapes
    if "result" in locs:
        val = locs["result"]
        if isinstance(val, (bd.Compound, bd.Solid, bd.Shape)):
            return val
        if isinstance(val, list) and all(
            isinstance(x, (bd.Compound, bd.Solid, bd.Shape)) for x in val
        ):
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


def parse_label(label: str, default_q: int, default_p: str) -> dict:
    data = {"quantity": default_q, "process": default_p}
    if not label:
        return data
    for p in str(label).split("|"):
        if ":" in p:
            k, v = p.split(":", 1)
            if k == "quantity":
                try:
                    data["quantity"] = int(v)
                except:
                    pass
            elif k == "process":
                data["process"] = v
    return data


def main():
    if len(sys.argv) < 2:
        print("Usage: python validation_runner.py <config_file>")
        sys.exit(1)

    config_path = sys.argv[1]
    output_path = "validation_result.json"

    # Initialize basic result structure in case of crash
    result_data = {
        "status": "error",
        "manufacturability_score": 0.0,
        "violations": [],
        "parts": [],
        "cost_analysis": {"total_cost": 0.0, "unit_cost": 0.0},
        "stl_path": None,
        "error": None,
    }

    try:
        with open(config_path, "r") as f:
            config = json.load(f)

        design_file = config.get("design_file", "design.py")
        process_default = config.get("process", "cnc")
        quantity_default = config.get("quantity", 1)
        export_stl_flag = config.get("export_stl", False)
        output_path = config.get("output_path", "validation_result.json")
        stl_filename = "design.stl"

        # Initialize Workbenches
        workbenches = {
            "cnc": CNCWorkbench(),
            "injection_molding": InjectionMoldingWorkbench(),
            "print_3d": Print3DWorkbench(),
        }

        # 1. execute code
        locs = {}
        with open(Path("/workspace") / design_file, "r", encoding="utf-8") as f:
            code = f.read()
        exec(code, globals(), locs)

        # 2. extract object
        export_obj = get_export_obj(locs)
        if not export_obj:
            raise ValueError("No 3D object found in script")

        all_solids = (
            list(export_obj.solids())
            if isinstance(export_obj, bd.Compound)
            else [export_obj]
        )

        total_cost = 0.0
        all_violations = []
        part_reports = []
        reuse_ctx = {}

        # 3. Analyze each part
        for i, solid in enumerate(all_solids):
            # Parse label for per-part overrides
            label_data = parse_label(
                getattr(solid, "label", ""), quantity_default, process_default
            )
            current_process = label_data["process"]
            current_quantity = label_data["quantity"]

            wb = workbenches.get(
                current_process,
                workbenches.get(process_default, workbenches["print_3d"]),
            )

            # Validate
            violations = wb.validate(solid)
            all_violations.extend(violations)

            # Cost
            cost_res = wb.calculate_cost(solid, current_quantity, context=reuse_ctx)

            # Convert dataclass to dict
            breakdown_dict = dataclasses.asdict(cost_res)
            # cost_res.total_cost is the cost for ALL units of this part
            total_cost += cost_res.total_cost

            part_reports.append(
                {
                    "part_index": i,
                    "process": current_process,
                    "quantity": current_quantity,
                    "cost": cost_res.total_cost,
                    "breakdown": breakdown_dict,
                    "violations": [str(v) for v in violations],
                }
            )

        # 4. Finalize Result
        status = "pass" if not all_violations else "fail"
        score = max(0.0, 1.0 - len(all_violations) * 0.1)

        # Unit cost is average across the requested default quantity, roughly
        # If we have mixed quantities, unit cost concept is fuzzy, but usually we consider total / default_quantity
        unit_cost = total_cost / quantity_default if quantity_default > 0 else 0.0

        result_data.update(
            {
                "status": status,
                "manufacturability_score": score,
                "violations": [{"description": str(v)} for v in all_violations],
                "parts": part_reports,
                "cost_analysis": {
                    "total_cost": total_cost,
                    "unit_cost": unit_cost,
                    # These are aggregated, so maybe meaningless, but required by schema structure?
                    # Evaluator re-wraps it in CostBreakdown later.
                },
                "error": None,
            }
        )

        # 5. Export STL if needed
        if export_stl_flag and not all_violations:
            stl_path = Path("/workspace") / stl_filename
            geometry.export_mesh(export_obj, str(stl_path))
            result_data["stl_path"] = stl_filename

    except Exception as e:
        result_data["error"] = f"{str(e)}"
        # result_data["error_trace"] = traceback.format_exc() # Optional debug

    # Write Result
    with open(f"/workspace/{output_path}", "w") as f:
        json.dump(result_data, f, indent=2)


if __name__ == "__main__":
    main()
