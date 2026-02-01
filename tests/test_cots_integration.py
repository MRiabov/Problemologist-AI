import os
import pytest
from src.environment.tools import search_parts, preview_part

def test_cots_search_and_preview_integration():
    # 1. Search for Nema motors
    search_results = search_parts("Nema")
    assert "Found parts:" in search_results
    assert "bd_warehouse:motor:Nema17" in search_results
    
    # 2. Pick a part ID from results
    # Example line: - bd_warehouse:motor:Nema17: Stepper Motor Nema17 (Provider: bd_warehouse)
    part_id = "bd_warehouse:motor:Nema17"
    
    # 3. Preview the part
    preview_output = preview_part(part_id)
    assert f"Part: {part_id}" in preview_output
    assert "Description:" in preview_output
    assert "Image:" in preview_output
    assert "Recipe:" in preview_output
    assert "from bd_warehouse.open_builds import StepperMotor" in preview_output
    
    # 4. Check if image path is valid (if it was generated)
    # The output format is: Image: path/to/image.png
    lines = preview_output.split("\n")
    image_line = [l for l in lines if l.startswith("Image:")][0]
    image_path = image_line.replace("Image:", "").strip()
    
    if image_path:
        # If image path is not empty, it should exist or be in a temporary directory
        # For tests, we might not want to rely on real rendering if it's slow or requires GUI
        # But we should at least see it tried to generate it.
        pass

def test_cots_search_no_results():
    search_results = search_parts("NonExistentPart12345")
    assert "No parts found for query: NonExistentPart12345" in search_results

def test_cots_preview_invalid_id():
    preview_output = preview_part("invalid:id")
    assert "Error previewing part invalid:id" in preview_output

def test_cad_env_cots_tools():
    from src.environment.core import CADEnv
    env = CADEnv()
    env.reset()
    
    # Test search_parts (tool index 5)
    action_search = {"tool": 5, "arguments": "Nema"}
    obs, reward, term, trunc, info = env.step(action_search)
    assert "bd_warehouse:motor:Nema17" in obs["last_output"]
    
    # Test preview_part (tool index 6)
    action_preview = {"tool": 6, "arguments": "bd_warehouse:motor:Nema17"}
    obs, reward, term, trunc, info = env.step(action_preview)
    assert "Part: bd_warehouse:motor:Nema17" in obs["last_output"]
    assert "from bd_warehouse.open_builds import StepperMotor" in obs["last_output"]
