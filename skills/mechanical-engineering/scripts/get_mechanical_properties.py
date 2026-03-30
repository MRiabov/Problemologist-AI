import json
from pathlib import Path

import yaml


def get_mechanical_data():
    """Reads manufacturing_config.yaml and prints material mechanical/FEM properties."""
    config_path = Path("config/manufacturing_config.yaml")
    if not config_path.exists():
        print(f"Error: Configuration not found at {config_path.absolute()}")
        return

    with config_path.open("r") as f:
        data = yaml.safe_load(f)
        materials = data.get("materials", {})

        mechanical_info = {}
        for mat_id, props in materials.items():
            mechanical_info[mat_id] = {
                "class": props.get("material_class"),
                "youngs_modulus_pa": props.get("youngs_modulus_pa"),
                "poissons_ratio": props.get("poissons_ratio"),
                "yield_stress_pa": props.get("yield_stress_pa"),
                "ultimate_stress_pa": props.get("ultimate_stress_pa"),
                "density_kg_m3": props.get("density_kg_m3"),
            }

        print(json.dumps(mechanical_info, indent=2))


if __name__ == "__main__":
    get_mechanical_data()
