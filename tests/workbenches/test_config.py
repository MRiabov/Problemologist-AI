import pytest
import pathlib
import structlog
from structlog.testing import capture_logs
from worker.workbenches.config import load_config
from worker.workbenches.models import ManufacturingConfig, ManufacturingMethod

def test_load_config():
    with capture_logs() as captured:
        config = load_config()
        assert any(log["event"] == "loading_manufacturing_config" for log in captured)
        assert any(log["event"] == "manufacturing_config_loaded" for log in captured)
    
    assert isinstance(config, ManufacturingConfig)
    assert config.cnc is not None
    assert "aluminum_6061" in config.cnc.materials
    
    # Check dict-like access for backward compatibility
    cnc_dict = config["cnc"]
    assert isinstance(cnc_dict, dict)
    assert "aluminum_6061" in cnc_dict["materials"]
    assert cnc_dict["materials"]["aluminum_6061"]["density_g_cm3"] == 2.7
    
    # Check rates in injection molding
    im_dict = config["injection_molding"]
    assert im_dict["materials"]["abs"]["machine_hourly_rate"] == 60.0

def test_config_file_not_found():
    with pytest.raises(FileNotFoundError):
        load_config("non_existent_file.yaml")

def test_invalid_yaml(tmp_path):
    invalid_file = tmp_path / "invalid.yaml"
    invalid_file.write_text("invalid: yaml: :")
    with pytest.raises(Exception):
        load_config(str(invalid_file))

def test_validation_error(tmp_path):
    invalid_data = tmp_path / "invalid_data.yaml"
    # cnc should be a dict, not a string
    invalid_data.write_text("cnc: 'not a dict'")
    with pytest.raises(Exception):
        load_config(str(invalid_data))