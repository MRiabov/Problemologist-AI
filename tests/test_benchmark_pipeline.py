import os
import shutil
import pytest
from unittest.mock import patch, MagicMock
from src.generators.benchmark.manager import generate, execute_build

# Mock script that defines a build function
MOCK_SCRIPT = """
def build(seed):
    return f'<mujoco><worldbody><geom name="box_{seed}" type="box" size="1 1 1"/></worldbody></mujoco>'
"""

@pytest.fixture
def clean_datasets():
    output_dir = "test_datasets"
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    yield output_dir
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)

def test_execute_build():
    mjcf = execute_build(MOCK_SCRIPT, 42)
    assert 'box_42' in mjcf
    assert '<mujoco>' in mjcf

@patch("src.generators.benchmark.manager.generator_agent.invoke")
def test_generate_pipeline(mock_invoke, clean_datasets):
    # Mock agent returning a valid script
    mock_invoke.return_value = {
        "validation_passed": True,
        "code": MOCK_SCRIPT,
        "request": "Test Prompt"
    }
    
    from typer.testing import CliRunner
    from src.generators.benchmark.manager import app
    
    runner = CliRunner()
    # When app has only one command, Typer might behave differently depending on how it's initialized.
    # Here 'generate' is a command, so we should call it.
    result = runner.invoke(app, [
        "--prompt", "Test Prompt",
        "--count", "2",
        "--output-dir", clean_datasets
    ])
    
    if result.exit_code != 0:
        print(result.output)
    assert result.exit_code == 0
    assert "Template generated successfully!" in result.output
    
    # Check if files were created
    # Find the generated directory (it has a random suffix)
    dirs = os.listdir(clean_datasets)
    assert len(dirs) == 1
    scenario_dir = os.path.join(clean_datasets, dirs[0])
    
    assert os.path.exists(os.path.join(scenario_dir, "template.py"))
    
    # We requested 2 variations
    files = os.listdir(scenario_dir)
    mjcf_files = [f for f in files if f.startswith("scene_")]
    manifest_files = [f for f in files if f.startswith("manifest_")]
    
    assert len(mjcf_files) == 2
    assert len(manifest_files) == 2
    
    # Check assets
    assets_dir = os.path.join(scenario_dir, "assets")
    assert os.path.exists(assets_dir)
    stl_files = os.listdir(assets_dir)
    assert len(stl_files) == 2
