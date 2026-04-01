import functools
from pathlib import Path
from typing import Any

import structlog
import yaml

from shared.workers.workbench_models import ManufacturingConfig

logger = structlog.get_logger()


def _default_config_path() -> Path:
    return Path(__file__).with_name("manufacturing_config.yaml")


def _read_config_data(config_path: Path) -> dict[str, Any]:
    if not config_path.exists():
        logger.error(
            "config_file_not_found", path=str(config_path), session_id="system"
        )
        raise FileNotFoundError(f"Config file not found: {config_path}")

    try:
        with config_path.open() as f:
            data = yaml.safe_load(f)
    except Exception as e:
        logger.warning("config_load_failed", error=str(e), session_id="system")
        raise

    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError("Manufacturing config YAML must deserialize to a mapping")
    return data


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in override.items():
        existing = merged.get(key)
        if isinstance(existing, dict) and isinstance(value, dict):
            merged[key] = _deep_merge(existing, value)
        else:
            merged[key] = value
    return merged


@functools.lru_cache
def load_config(config_path: str | None = None) -> ManufacturingConfig:
    if config_path is None:
        config_path = _default_config_path()
    else:
        config_path = Path(config_path)

    logger.info("loading_manufacturing_config", path=str(config_path))
    data = _read_config_data(config_path)

    try:
        config = ManufacturingConfig(**data)
        methods = [
            m
            for m in ["cnc", "injection_molding", "three_dp"]
            if getattr(config, m) is not None
        ]
        logger.info("manufacturing_config_loaded", methods=methods, session_id="system")
        return config
    except Exception as e:
        logger.warning("config_validation_failed", error=str(e), session_id="system")
        raise


def load_merged_config(
    config_path: str | Path | None = None,
    override_data: dict[str, Any] | None = None,
) -> ManufacturingConfig:
    if config_path is None and override_data is None:
        return load_config()

    base_config = load_config().model_dump(mode="python")
    merged_data = dict(base_config)

    if config_path is not None:
        merged_data = _deep_merge(merged_data, _read_config_data(Path(config_path)))
    if override_data is not None:
        if not isinstance(override_data, dict):
            raise ValueError("override_data must be a mapping when provided")
        merged_data = _deep_merge(merged_data, override_data)

    try:
        config = ManufacturingConfig.model_validate(merged_data)
        methods = [
            m
            for m in ["cnc", "injection_molding", "three_dp"]
            if getattr(config, m) is not None
        ]
        logger.info(
            "manufacturing_config_merged_loaded",
            methods=methods,
            override_path=str(config_path) if config_path is not None else None,
            session_id="system",
        )
        return config
    except Exception as e:
        logger.warning(
            "merged_config_validation_failed", error=str(e), session_id="system"
        )
        raise
