import json
import pytest
from pathlib import Path
from scripts.export_container_schema import app


def test_container_schema_is_up_to_date():
    """
    Verifies that the stored container_schema.json is up-to-date with the
    current state of the container_agent application.
    """
    project_root = Path(__file__).resolve().parent.parent
    schema_path = project_root / "src" / "environment" / "container_schema.json"

    # Generate current schema
    current_schema = app.openapi()

    # Ensure the schema file exists
    assert schema_path.exists(), (
        f"Schema file not found at {schema_path}. Run scripts/export_container_schema.py to generate it."
    )

    # Load stored schema
    with open(schema_path, "r") as f:
        stored_schema = json.load(f)

    # Compare schemas
    assert current_schema == stored_schema, (
        "The stored container_schema.json is out of date. "
        "Please run 'python scripts/export_container_schema.py' to update it."
    )

    # Check if gen_models.py exists
    models_path = project_root / "src" / "assets" / "scripts" / "gen_models.py"
    assert models_path.exists(), (
        f"Generated models not found at {models_path}. Run scripts/export_container_schema.py."
    )
