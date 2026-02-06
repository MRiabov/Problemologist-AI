import pathlib
from typing import Optional
import yaml
import structlog
from src.workbenches.models import ManufacturingConfig

logger = structlog.get_logger()

def load_config(config_path: Optional[str] = None) -> ManufacturingConfig:
    """
    Loads and validates the manufacturing configuration from a YAML file.
    
    Args:
        config_path: Path to the YAML config file. Defaults to manufacturing_config.yaml
                     in the same directory.
                     
    Returns:
        A validated ManufacturingConfig Pydantic model.
    """
    if config_path is None:
        # Default path relative to this file
        config_path = pathlib.Path(__file__).parent / "manufacturing_config.yaml"
    else:
        config_path = pathlib.Path(config_path)

    logger.info("loading_manufacturing_config", path=str(config_path))

    if not config_path.exists():
        logger.error("config_file_not_found", path=str(config_path))
        raise FileNotFoundError(f"Config file not found: {config_path}")

    try:
        with open(config_path, "r") as f:
            data = yaml.safe_load(f)
    except Exception as e:
        logger.error("config_load_failed", error=str(e))
        raise

    try:
        config = ManufacturingConfig(**data)
        methods = [m for m in ["cnc", "injection_molding", "three_dp"] if getattr(config, m) is not None]
        logger.info("manufacturing_config_loaded", methods=methods)
        return config
    except Exception as e:
        logger.error("config_validation_failed", error=str(e))
        raise
