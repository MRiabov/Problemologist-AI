import json
import sys
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.append(str(project_root))

from controller.api.main import app as controller_app  # noqa: E402
from worker_heavy.app import app as worker_heavy_app  # noqa: E402
from worker_light.app import app as worker_light_app  # noqa: E402


def generate_schema(app, filename):
    schema = app.openapi()
    output_path = project_root / filename
    with output_path.open("w") as f:
        json.dump(schema, f, indent=2)
    print(f"Generated {filename}")


def generate_merged_worker_schema():
    light_schema = worker_light_app.openapi()
    heavy_schema = worker_heavy_app.openapi()

    # Merge paths and components
    light_schema["paths"].update(heavy_schema.get("paths", {}))
    if "components" in heavy_schema:
        if "components" not in light_schema:
            light_schema["components"] = {}
        if "schemas" in heavy_schema["components"]:
            if "schemas" not in light_schema["components"]:
                light_schema["components"]["schemas"] = {}
            light_schema["components"]["schemas"].update(
                heavy_schema["components"]["schemas"]
            )

    output_path = project_root / "worker_openapi.json"
    with output_path.open("w") as f:
        json.dump(light_schema, f, indent=2)
    print("Generated merged worker_openapi.json")


if __name__ == "__main__":
    generate_schema(controller_app, "controller_openapi.json")
    generate_merged_worker_schema()
