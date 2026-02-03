import yaml
from pathlib import Path
import json

def get_data():
    config_path = Path("config/manufacturing_config.yaml")
    if not config_path.exists():
        print(f"Error: Configuration not found at {config_path.absolute()}")
        return

    with config_path.open("r") as f:
        data = yaml.safe_load(f)
        # Output as JSON for easy parsing if needed, or just pretty print
        print(json.dumps(data, indent=2))

if __name__ == "__main__":
    get_data()
