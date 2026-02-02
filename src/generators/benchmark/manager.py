import os
import random
import json
import uuid
from typing import List, Dict, Any, Union
from pathlib import Path
import yaml
import typer
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn

from src.generators.benchmark.agent import generator_agent
from src.generators.benchmark.validator import validate_mjcf
from src.generators.benchmark.renderer import render_scenario
from src.generators.benchmark.types import ScenarioManifest, ValidationReport
from src.environment.sandbox import PodmanSandbox

# Load config
GEN_CONFIG_PATH = Path(__file__).parent / "generator_config.yaml"
with GEN_CONFIG_PATH.open("r") as f:
    gen_config = yaml.safe_load(f)

HEADROOM_FACTOR = gen_config.get("headroom_factor", 3)

app = typer.Typer(help="Benchmark Scenario Generator CLI")
console = Console()


def execute_build(
    code: str, seed: int, scale_factors: tuple[float, float, float] = (1.0, 1.0, 1.0), asset_dir: str = None
) -> str:
    """Executes the build(seed, scale_factors) function from the provided code inside a sandbox."""
    # Prepend hardcoded imports and helpers
    final_code = (
        "from build123d import *\n"
        "from build123d.topology import Shape\n"
        "from typing import Union\n"
        "from src.simulation_engine.builder import SceneCompiler\n\n"
        "_ASSET_DIR = None\n\n"
        "def to_mjcf(env_compound: Union[Compound, Shape, list], agent_compound: Union[Compound, Shape, list] = None, agent_joints: list = None, env_labels: list[str] = None, agent_labels: list[str] = None) -> str:\n"
        "    from build123d import Compound\n"
        "    from build123d.topology import Shape\n"
        "    # Use fixed absolute path for sandbox consistency\n"
        "    target_dir = _ASSET_DIR or '/workspace/.agent_storage/temp_assets'\n"
        "    import os\n"
        "    os.makedirs(target_dir, exist_ok=True)\n"
        "    compiler = SceneCompiler(asset_dir=target_dir)\n"
        "    \n"
        "    def ensure_compound(obj):\n"
        "        if obj is None: return None\n"
        "        if isinstance(obj, list): return Compound(children=obj)\n"
        "        if isinstance(obj, (Compound, Shape)): return obj\n"
        "        return Compound(children=[obj])\n"
        "        \n"
        "    env_c = ensure_compound(env_compound)\n"
        "    agent_c = ensure_compound(agent_compound)\n"
        "    return compiler.compile(env_c, agent_c, agent_joints, env_labels, agent_labels)\n\n"
        + code
    )

    # Initialize a temporary workspace for generation
    workspace = os.path.abspath("workspace_gen")
    os.makedirs(workspace, exist_ok=True)
    
    # Ensure asset_dir exists in workspace if provided, and CLEAR it if it already has files
    if asset_dir:
        host_asset_path = os.path.join(workspace, asset_dir)
        if os.path.exists(host_asset_path):
            import shutil
            shutil.rmtree(host_asset_path)
        os.makedirs(host_asset_path, exist_ok=True)

    sandbox = PodmanSandbox(workspace)

    script_name = "template_build.py"
    runner_name = "runner_build.py"

    with open(os.path.join(workspace, script_name), "w") as f:
        f.write(final_code)

    asset_dir_val = f"'{asset_dir}'" if asset_dir else "None"

    runner_script = f"""
import json
import sys
import os

sys.path.append("/workspace")
import template_build

# Inject asset_dir as relative path for SceneCompiler logic
template_build._ASSET_DIR = {asset_dir_val}

seed = {seed}
scale_factors = {scale_factors}

try:
    import inspect
    sig = inspect.signature(template_build.build)
    if "scale_factors" in sig.parameters:
        res = template_build.build(seed, scale_factors=scale_factors)
    elif "scale" in sig.parameters:
        res = template_build.build(seed, scale=scale_factors)
    else:
        res = template_build.build(seed)
    print(f"BUILD_RESULT:{{res}}")
except Exception as e:
    import traceback
    print(f"BUILD_ERROR:{{str(e)}}")
    print(traceback.format_exc())
"""

    try:
        with open(os.path.join(workspace, runner_name), "w") as f:
            f.write(runner_script)

        stdout, stderr, rc = sandbox.run_script(runner_name, mount_src=True)

        if "BUILD_RESULT:" in stdout:
            # Extract XML
            return stdout.split("BUILD_RESULT:")[1].strip()
        else:
            error = stdout + stderr
            raise ValueError(f"Build execution failed: {error}")
    finally:
        # Clean up
        if os.path.exists(os.path.join(workspace, runner_name)):
            os.remove(os.path.join(workspace, runner_name))
        if os.path.exists(os.path.join(workspace, script_name)):
            os.remove(os.path.join(workspace, script_name))


@app.command()
def generate(
    prompt: str = typer.Option(
        ..., "--prompt", "-p", help="The description of the scenario to generate"
    ),
    count: int = typer.Option(
        1, "--count", "-c", help="Number of variations to generate"
    ),
    output_dir: str = typer.Option(
        "datasets/benchmarks",
        "--output-dir",
        "-o",
        help="Root directory for generated datasets",
    ),
    tier: str = typer.Option(
        "spatial",
        "--tier",
        "-t",
        help="Tier of the scenario (e.g., spatial, kinematic)",
    ),
):
    """
    Generates a benchmark scenario template using the AI agent, then produces randomized variations.
    """
    console.print(f"[bold blue]Generating Scenario Template for:[/bold blue] {prompt}")

    # 1. Invoke Agent to get template script
    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        transient=True,
    ) as progress:
        progress.add_task(description="Agent thinking...", total=None)
        state = generator_agent.invoke({"request": prompt, "attempts": 0})

    if not state.get("validation_passed"):
        console.print(
            "[bold red]Error:[/bold red] Agent failed to produce a valid script."
        )
        if state.get("errors"):
            console.print(f"Details: {state['errors']}")
        return

    template_code = state["code"]
    scenario_id = str(uuid.uuid4())[:8]
    scenario_dir = os.path.join(output_dir, f"{tier}_{scenario_id}")
    os.makedirs(scenario_dir, exist_ok=True)
    os.makedirs(os.path.join(scenario_dir, "assets"), exist_ok=True)
    os.makedirs(os.path.join(scenario_dir, "images"), exist_ok=True)

    # Save the template script
    script_path = os.path.join(scenario_dir, "template.py")
    with open(script_path, "w") as f:
        f.write(template_code)

    console.print(
        f"[bold green]Template generated successfully![/bold green] (ID: {scenario_id})"
    )
    console.print(f"Entering Mass Production phase for {count} variations...")

    valid_variations = 0
    attempts = 0
    max_attempts = count * HEADROOM_FACTOR  # Allow some headroom for failures

    with Progress() as progress:
        task = progress.add_task("[cyan]Producing variations...", total=count)

        while valid_variations < count and attempts < max_attempts:
            attempts += 1
            seed = random.randint(0, 1000000)

            # Generate random non-uniform scale (0.5 to 2.0)
            sx = random.uniform(0.5, 2.0)
            sy = random.uniform(0.5, 2.0)
            sz = random.uniform(0.5, 2.0)
            scale_factors = (sx, sy, sz)

            try:
                # 2. Batch Processing & Randomization
                # We use a temporary assets dir for validation
                rel_temp_assets = ".agent_storage/temp_assets"
                print(f"DEBUG: Executing build for seed {seed}")
                mjcf_xml = execute_build(template_code, seed, scale_factors=scale_factors, asset_dir=rel_temp_assets)

                # 3. Validation
                print(f"DEBUG: Validating MJCF for seed {seed}")
                temp_assets_path = os.path.join("workspace_gen", rel_temp_assets)
                report = validate_mjcf(mjcf_xml, asset_dir=temp_assets_path)

                if report["is_valid"]:
                    print(f"DEBUG: Seed {seed} is valid, saving artifacts")
                    # 4. Artifact Export
                    variation_id = f"var_{seed}"

                    # Save MJCF
                    xml_path = os.path.join(scenario_dir, f"scene_{seed}.xml")
                    with open(xml_path, "w") as f:
                        f.write(mjcf_xml)

                    # Move temp assets to final assets
                    temp_assets_path = os.path.join("workspace_gen", rel_temp_assets)
                    final_assets_path = os.path.join(scenario_dir, "assets")
                    generated_meshes = []
                    if os.path.exists(temp_assets_path):
                        import shutil
                        for mesh_file in os.listdir(temp_assets_path):
                            shutil.move(os.path.join(temp_assets_path, mesh_file), os.path.join(final_assets_path, mesh_file))
                            generated_meshes.append(f"assets/{mesh_file}")

                    # Render images
                    image_prefix = os.path.join(
                        scenario_dir, "images", f"preview_{seed}"
                    )
                    image_paths = render_scenario(mjcf_xml, image_prefix)
                    # Convert absolute paths back to relative for manifest
                    rel_image_paths = [
                        os.path.relpath(p, scenario_dir) for p in image_paths
                    ]

                    # Save Manifest
                    manifest: ScenarioManifest = {
                        "id": f"{scenario_id}_{seed}",
                        "tier": tier,
                        "description": prompt,
                        "script_path": "template.py",
                        "assets": {
                            "mjcf": f"scene_{seed}.xml",
                            "meshes": generated_meshes,
                            "images": rel_image_paths,
                        },
                        "randomization": {
                            "seed_range": [0, 1000000],
                            "parameters": ["seed", "scale"],
                            "scale": [sx, sy, sz],
                        },
                        "validation": {
                            "passed": True,
                            "max_velocity": 0.0,  # Should extract from report if available
                        },
                    }

                    manifest_path = os.path.join(scenario_dir, f"manifest_{seed}.json")
                    with open(manifest_path, "w") as f:
                        json.dump(manifest, f, indent=2)

                    valid_variations += 1
                    progress.update(task, advance=1)
                else:
                    print(f"DEBUG: Seed {seed} FAILED validation: {report['error_message']}")
                    console.print(
                        f"[yellow]Variation with seed {seed} failed validation: {report['error_message']}[/yellow]"
                    )
            except Exception as e:
                console.print(
                    f"[red]Error producing variation with seed {seed}: {e}[/red]"
                )

    if valid_variations == count:
        console.print(
            f"\n[bold green]Success![/bold green] Generated {count} variations in {scenario_dir}"
        )
    else:
        console.print(
            f"\n[bold yellow]Completed with warnings:[/bold yellow] Only generated {valid_variations}/{count} variations."
        )


if __name__ == "__main__":
    app()
