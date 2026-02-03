import lxml.etree as ET
from pathlib import Path
from src.utils.paths import get_assets_dir


def validate_mjcf(xml_string: str) -> list[str]:
    """
    Validates an MJCF XML string against the XSD schema.
    Returns a list of error messages, or an empty list if valid.
    """
    schema_path = get_assets_dir() / "schemas" / "mjcf" / "mujoco.xsd"
    if not schema_path.exists():
        return [f"MJCF Schema not found at {schema_path}"]

    try:
        # Load the schema
        schema_doc = ET.parse(str(schema_path))
        schema = ET.XMLSchema(schema_doc)

        # Parse the XML string
        xml_doc = ET.fromstring(xml_string.encode("utf-8"))

        # Validate
        schema.assertValid(xml_doc)
        return []
    except ET.DocumentInvalid as e:
        return [str(error) for error in e.error_log]
    except Exception as e:
        return [f"Unexpected error during MJCF validation: {e!s}"]
