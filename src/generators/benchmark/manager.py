import json
import logging
import random
import shutil
import uuid
from pathlib import Path

import typer
import yaml
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn

from src.agent.utils.config import Config
from src.environment.sandbox import PodmanSandbox
from src.environment.sandbox_utils import run_sandboxed_script
from src.generators.benchmark.agent import generator_agent
from src.generators.benchmark.renderer import render_scenario
from src.generators.benchmark.types import ScenarioManifest
from src.generators.benchmark.validator import validate_mjcf

# Load config
GEN_CONFIG_PATH = Config.PROJECT_ROOT / "config" / "generator_config.yaml"
with GEN_CONFIG_PATH.open("r") as f:
    gen_config = yaml.safe_load(f)

HEADROOM_FACTOR = gen_config.get("headroom_factor", 3)

app = typer.Typer(help="Benchmark Scenario Generator CLI")
console = Console()
logger = logging.getLogger(__name__)


def execute_build(
    code: str,
    seed: int,
    scale_factors: tuple[float, float, float] = (1.0, 1.0, 1.0),
    variation_id: str = None,
) -> str:
    """Executes the build(seed, scale_factors) function from the provided code inside a sandbox."""
    # Use unique asset directory for this variation
    variation_id = variation_id or str(uuid.uuid4())[:8]
    asset_dir = f".agent_storage/temp_assets_{variation_id}"
    result_file = f"gen_result_{variation_id}.json"

    # Prepend hardcoded imports and helpers
    final_code = (
        "from build123d import *\n"
        "from build123d.topology import Shape\n"
        "from typing import Union\n"
        "from src.simulation_engine.builder import SceneCompiler\n\n"
        f"_ASSET_DIR = '{asset_dir}'\n\n"
        "def to_mjcf(env_compound: Union[Compound, Shape, list], agent_compound: Union[Compound, Shape, list] = None, agent_joints: list = None, env_labels: list[str] = None, agent_labels: list[str] = None) -> str:\n"
        "    from build123d import Compound\n"
        "    from build123d.topology import Shape\n"
        "    target_dir = f'/workspace/{{_ASSET_DIR}}'\n"
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
    workspace = Config.GEN_WORKSPACE_DIR
    workspace.mkdir(parents=True, exist_ok=True)

    # Ensure unique asset_dir exists in workspace
    host_asset_path = workspace / asset_dir
    if host_asset_path.exists():
        shutil.rmtree(host_asset_path)
    host_asset_path.mkdir(parents=True, exist_ok=True)

    sandbox = PodmanSandbox(str(workspace))

    runner_script = f"""
import json
import sys
import os

sys.path.append("/workspace")
import template_build

# Inject asset_dir
template_build._ASSET_DIR = '{asset_dir}'

seed = {seed}
scale_factors = {scale_factors}

result = {{"mjcf": None, "error": None}}

try:
    import inspect
    sig = inspect.signature(template_build.build)
    if "scale_factors" in sig.parameters:
        mjcf = template_build.build(seed, scale_factors=scale_factors)
    elif "scale" in sig.parameters:
        mjcf = template_build.build(seed, scale=scale_factors)
    else:
        mjcf = template_build.build(seed)
    result["mjcf"] = mjcf
except Exception as e:
    import traceback
    result["error"] = str(e) + "\n" + traceback.format_exc()

with open("/workspace/{result_file}", "w") as f:
    json.dump(result, f)
"""

    try:
        (workspace / "template_build.py").write_text(final_code)

        res = run_sandboxed_script(
            sandbox=sandbox,
            script_content=runner_script,
            result_file_name=result_file,
            runner_file_name=f"runner_{variation_id}.py",
        )

        if "mjcf" in res and res["mjcf"]:
            return res["mjcf"], asset_dir
        
        error = res.get("error") or res.get("message") or "Unknown error"
        raise ValueError(f"Build execution failed: {error}")
    finally:
        # Clean up template script
        script_path = workspace / "template_build.py"
        if script_path.exists():
            script_path.unlink()


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
    target_quantity: int = typer.Option(
        1, "--quantity", "-q", help="Target production quantity for the solution"
    ),
    max_unit_cost: float = typer.Option(
        1000.0, "--max-cost", help="Maximum allowable unit cost for the solution"
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
        # Pass cost constraints in the request
        full_prompt = (
            f"{prompt}\n\n"
            f"Economic Constraints:\n"
            f"- Target Quantity: {target_quantity}\n"
            f"- Max Unit Cost: ${max_unit_cost}"
        )
        state = generator_agent.invoke({"request": full_prompt, "attempts": 0})

    if not state.get("validation_passed"):
        console.print(
            "[bold red]Error:[/bold red] Agent failed to produce a valid script."
        )
        if state.get("errors"):
            console.print(f"Details: {state['errors']}")
        return

    template_code = state["code"]
    scenario_id = str(uuid.uuid4())[:8]
    scenario_dir = Path(output_dir) / f"{tier}_{scenario_id}"
    scenario_dir.mkdir(parents=True, exist_ok=True)
    (scenario_dir / "assets").mkdir(parents=True, exist_ok=True)
    (scenario_dir / "images").mkdir(parents=True, exist_ok=True)

    # Save the template script
    script_path = scenario_dir / "template.py"
    script_path.write_text(template_code)

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
                variation_id = f"var_{seed}"
                logger.debug(f"Executing build for seed {seed}")
                mjcf_xml, rel_temp_assets = execute_build(
                    template_code,
                    seed,
                    scale_factors=scale_factors,
                    variation_id=variation_id,
                )

                # 3. Validation
                logger.debug(f"Validating MJCF for seed {seed}")
                temp_assets_path = Config.GEN_WORKSPACE_DIR / rel_temp_assets
                report = validate_mjcf(mjcf_xml, asset_dir=str(temp_assets_path))

                if report["is_valid"]:
                    logger.debug(f"Seed {seed} is valid, saving artifacts")
                    # 4. Artifact Export

                    # Save MJCF
                    xml_path = scenario_dir / f"scene_{seed}.xml"
                    xml_path.write_text(mjcf_xml)

                    # Move temp assets to final assets
                    final_assets_path = scenario_dir / "assets"
                    generated_meshes = []
                    if temp_assets_path.exists():
                        for mesh_file in temp_assets_path.iterdir():
                            shutil.move(
                                str(mesh_file),
                                str(final_assets_path / mesh_file.name),
                            )
                            generated_meshes.append(f"assets/{mesh_file.name}")
                        
                        # Cleanup unique temp assets dir
                        shutil.rmtree(temp_assets_path)

                    # Render images
                    image_prefix = str(scenario_dir / "images" / f"preview_{seed}")
                    image_paths = render_scenario(mjcf_xml, image_prefix)
                    # Convert absolute paths back to relative for manifest
                    rel_image_paths = [
                        str(Path(p).relative_to(scenario_dir)) for p in image_paths
                    ]

                    # Save Manifest
                    manifest: ScenarioManifest = {
                        "id": f"{scenario_id}_{seed}",
                        "tier": tier,
                        "description": prompt,
                        "script_path": "template.py",
                        "target_quantity": target_quantity,
                        "max_unit_cost": max_unit_cost,
                        "cost_record": None,  # No record yet
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

                    manifest_path = scenario_dir / f"manifest_{seed}.json"
                    with manifest_path.open("w") as f:
                        json.dump(manifest, f, indent=2)

                    valid_variations += 1
                    progress.update(task, advance=1)
                else:
                    logger.debug(
                        f"Seed {seed} FAILED validation: {report['error_message']}"
                    )
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
