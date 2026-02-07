import json
import os
import sys
from pathlib import Path

# Add project root and src to path
project_root = Path(__file__).parent.parent
sys.path.append(str(project_root))
sys.path.append(str(project_root / "src"))

from controller.api.main import app as controller_app
from worker.app import app as worker_app

def generate_schema(app, filename):
    schema = app.openapi()
    output_path = project_root / filename
    with open(output_path, "w") as f:
        json.dump(schema, f, indent=2)
    print(f"Generated {filename}")

if __name__ == "__main__":
    generate_schema(controller_app, "controller_openapi.json")
    generate_schema(worker_app, "worker_openapi.json")
