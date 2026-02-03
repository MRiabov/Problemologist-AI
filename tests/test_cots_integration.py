from src.environment.runtime import ToolRuntime


def test_cots_search_and_preview_integration(tmp_path):
    runtime = ToolRuntime(workspace_dir=str(tmp_path / "workspace"))

    # 1. Search for Nema motors
    search_results = runtime.dispatch("search_parts", {"query": "Nema"})
    assert "Found parts:" in search_results
    assert "bd_warehouse:motor:Nema17" in search_results

    # 2. Pick a part ID from results
    part_id = "bd_warehouse:motor:Nema17"

    # 3. Preview the part
    preview_output = runtime.dispatch("preview_part", {"part_id": part_id})
    assert f"Part: {part_id}" in preview_output
    assert "Recipe:" in preview_output
    assert "from bd_warehouse.open_builds import StepperMotor" in preview_output

    # 4. Check image path logic (if applicable in runtime implementation)
    # The runtime implementation currently returns a string description/recipe.
    # If it generated an image, it would be in the description.
    pass


def test_cots_search_no_results(tmp_path):
    runtime = ToolRuntime(workspace_dir=str(tmp_path / "workspace"))
    search_results = runtime.dispatch("search_parts", {"query": "NonExistentPart12345"})
    assert "No parts found for: NonExistentPart12345" in search_results


def test_cots_preview_invalid_id(tmp_path):
    runtime = ToolRuntime(workspace_dir=str(tmp_path / "workspace"))
    preview_output = runtime.dispatch("preview_part", {"part_id": "invalid:id"})
    assert "Error:" in preview_output


def test_runtime_cots_tools(tmp_path):
    from src.environment.runtime import ToolRuntime

    workspace_dir = tmp_path / "workspace"
    runtime = ToolRuntime(workspace_dir=str(workspace_dir))

    # Test search_parts (tool index 5)
    # env.step -> runtime.dispatch
    search_output = runtime.dispatch("search_parts", {"query": "Nema"})
    assert "bd_warehouse:motor:Nema17" in search_output

    # Test preview_part (tool index 6)
    preview_output = runtime.dispatch(
        "preview_part", {"part_id": "bd_warehouse:motor:Nema17"}
    )
    assert "Part: bd_warehouse:motor:Nema17" in preview_output
    assert "from bd_warehouse.open_builds import StepperMotor" in preview_output
