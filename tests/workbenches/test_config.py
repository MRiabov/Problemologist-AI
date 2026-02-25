import pytest
from structlog.testing import capture_logs

from shared.workers.workbench_models import ManufacturingConfig
from worker_heavy.workbenches.config import load_config


def test_load_config():
    with capture_logs() as captured:
        config = load_config()
        assert any(log["event"] == "loading_manufacturing_config" for log in captured)
        assert any(log["event"] == "manufacturing_config_loaded" for log in captured)

    assert isinstance(config, ManufacturingConfig)
    assert config.cnc is not None
    assert "aluminum_6061" in config.cnc.materials

    # Verify new fields for simulation
    al_mat = config.cnc.materials["aluminum_6061"]
    assert al_mat.color == "#D1D5DB"
    assert al_mat.elongation_stress_mpa == 240.0
    assert al_mat.restitution == 0.5
    assert al_mat.friction_coef == 0.6

    # Check dict-like access for backward compatibility
    cnc_dict = config["cnc"]
    assert isinstance(cnc_dict, dict)
    assert "aluminum_6061" in cnc_dict["materials"]
    assert cnc_dict["materials"]["aluminum_6061"]["density_g_cm3"] == 2.7
    assert cnc_dict["materials"]["aluminum_6061"]["color"] == "#D1D5DB"

    # Check rates in injection molding
    im_dict = config["injection_molding"]
    assert im_dict["materials"]["abs"]["machine_hourly_rate"] == 60.0
    assert im_dict["materials"]["abs"]["restitution"] == 0.4


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
