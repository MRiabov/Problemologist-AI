import json
import subprocess
import sys
from pathlib import Path

# Add project root to path
project_root = Path(__file__).resolve().parent.parent
sys.path.append(str(project_root))

from src.assets.scripts.container_agent import app


def export_schema():
    openapi_schema = app.openapi()

    schema_path = project_root / "src" / "environment" / "container_schema.json"
    models_path = project_root / "src" / "assets" / "scripts" / "gen_models.py"

    # Export Schema
    with schema_path.open("w") as f:
        json.dump(openapi_schema, f, indent=2)
    print(f"Schema exported to {schema_path}")

    # Generate Models
    print(f"Generating models to {models_path}...")
    try:
        subprocess.run(
            [
                "uv",
                "run",
                "datamodel-codegen",
                "--input",
                str(schema_path),
                "--input-file-type",
                "openapi",
                "--output",
                str(models_path),
                "--output-model-type",
                "pydantic_v2.BaseModel",
                "--enum-field-as-literal",
                "one",
                # Capitalize members because OpenAPI enum only stores values (e.g. "success"),
                # so the generator needs a hint to use uppercase names (e.g. SUCCESS).
                "--capitalise-enum-members",
                "--use-specialized-enum",
                "--target-python-version",
                "3.12",
                "--use-schema-description",
                "--disable-timestamp",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        print("Models generated successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error generating models: {e.stderr}")
        sys.exit(1)


if __name__ == "__main__":
    export_schema()
