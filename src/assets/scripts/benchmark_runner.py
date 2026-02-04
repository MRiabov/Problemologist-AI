import importlib
import inspect
import json
import os
import sys
import traceback
from pathlib import Path


def main():
    if len(sys.argv) < 2:
        config_path = Path("/workspace/benchmark_config.json")
    else:
        config_path = Path(sys.argv[1])

    result = {"mjcf": None, "error": None}

    try:
        if not config_path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")

        with config_path.open("r") as f:
            config = json.load(f)

        seed = config.get("seed", 0)
        scale_factors = config.get("scale_factors", (1.0, 1.0, 1.0))
        asset_dir = config.get("asset_dir", ".agent_storage/temp_assets")
        result_file = config.get("result_file", "gen_result.json")
        module_name = config.get("module_name", "template_build")

        sys.path.append("/workspace")

        try:
            template_build = importlib.import_module(module_name)
        except ImportError:
            # Fallback for older patterns
            import template_build

        # Inject asset_dir
        template_build._ASSET_DIR = asset_dir
        os.environ["BENCHMARK_ASSET_DIR"] = asset_dir

        sig = inspect.signature(template_build.build)
        if "scale_factors" in sig.parameters:
            mjcf = template_build.build(seed, scale_factors=scale_factors)
        elif "scale" in sig.parameters:
            mjcf = template_build.build(seed, scale=scale_factors)
        else:
            mjcf = template_build.build(seed)

        result["mjcf"] = mjcf

    except Exception as e:
        result["error"] = f"{e!s}\n{traceback.format_exc()}"

    # Write Result
    output_path = Path("/workspace") / result_file
    with output_path.open("w") as f:
        json.dump(result, f)


if __name__ == "__main__":
    main()
