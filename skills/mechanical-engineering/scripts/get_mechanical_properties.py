import json
from pathlib import Path

import yaml


def _find_config_path() -> Path | None:
    for candidate in (
        Path("manufacturing_config.yaml"),
        Path("config/manufacturing_config.yaml"),
    ):
        if candidate.exists():
            return candidate
    return None


def get_mechanical_data():
    """Reads the workspace manufacturing config and prints material properties."""
    config_path = _find_config_path()
    if config_path is None:
        print(
            "Error: Configuration not found at manufacturing_config.yaml or config/manufacturing_config.yaml"
        )
        return

    with config_path.open("r") as f:
        data = yaml.safe_load(f) or {}
        materials = data.get("materials", {})

        mechanical_info = {}
        for mat_id, props in materials.items():
            mechanical_info[mat_id] = {
                "class": props.get("material_class"),
                "friction_coef": props.get("friction_coef"),
                "restitution": props.get("restitution"),
                "youngs_modulus_pa": props.get("youngs_modulus_pa"),
                "poissons_ratio": props.get("poissons_ratio"),
                "yield_stress_pa": props.get("yield_stress_pa"),
                "ultimate_stress_pa": props.get("ultimate_stress_pa"),
                "density_kg_m3": props.get("density_kg_m3"),
            }

        print(json.dumps(mechanical_info, indent=2))


if __name__ == "__main__":
    get_mechanical_data()
