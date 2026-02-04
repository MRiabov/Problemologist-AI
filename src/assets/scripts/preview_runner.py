import sys
import json
import traceback
from pathlib import Path
from typing import Any

import build123d as bd
from build123d import ExportSVG, Drawing, Unit

# Append workspace to sys.path to allow importing local design files
sys.path.append("/workspace")


def get_export_obj(locs: dict[str, Any]) -> Any:
    """Extracts the object to export from local variables."""
    # Prefer 'result' if it exists and is a valid shape or list of shapes
    if "result" in locs:
        val = locs["result"]
        if isinstance(val, (bd.Compound, bd.Solid, bd.Shape)):
            return val
        if isinstance(val, list) and all(
            isinstance(x, (bd.Compound, bd.Solid, bd.Shape)) for x in val
        ):
            return bd.Compound(val)

    # Otherwise scan locals for shapes
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


def main():
    """
    Main entry point for the preview runner.
    Expected usage: python preview_runner.py <json_config_path>
    """
    if len(sys.argv) < 2:
        config_path = "/workspace/job_config.json"
    else:
        config_path = sys.argv[1]

    # Default result structure
    result = {"status": "error", "preview_file": None, "error": None}

    output_path = "preview_result.json"

    try:
        with open(config_path, "r") as f:
            config = json.load(f)

        filename = config.get("filename", "design.py")
        output_path = config.get("output_path", "preview_result.json")

        # Check if file exists
        design_path = Path("/workspace") / filename
        if not design_path.exists():
            raise FileNotFoundError(f"Design file {filename} not found in workspace")

        # Execute the user script
        locs = {}
        with open(design_path, "r", encoding="utf-8") as f:
            code = f.read()

        exec(code, globals(), locs)

        # Extract object
        export_obj = get_export_obj(locs)

        if export_obj:
            svg_filename = f"{Path(filename).stem}.svg"
            svg_path = Path("/workspace") / svg_filename

            try:
                drawing = Drawing(export_obj)
                exporter = ExportSVG(unit=Unit.MM)
                exporter.add_layer("visible", line_color=(0, 0, 0), line_weight=0.2)
                exporter.add_shape(drawing.visible_lines, layer="visible")
                exporter.write(str(svg_path))

                result["status"] = "success"
                result["preview_file"] = svg_filename
            except Exception as e:
                result["error"] = f"SVG generation failed: {str(e)}"
        else:
            result["error"] = "No 3D object found in script"

    except Exception as e:
        result["error"] = f"Runtime error: {str(e)}\n{traceback.format_exc()}"

    # Write result
    try:
        with open(f"/workspace/{output_path}", "w") as f:
            json.dump(result, f, indent=2)
    except Exception as e:
        # Fallback if writing result fails
        print(f"Critical error writing result: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
