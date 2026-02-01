import os
import random
import json
import uuid
from typing import List, Dict, Any
import typer
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn

from src.generators.benchmark.agent import generator_agent
from src.generators.benchmark.validator import validate_mjcf
from src.generators.benchmark.renderer import render_scenario
from src.generators.benchmark.types import ScenarioManifest, ValidationReport

app = typer.Typer(help="Benchmark Scenario Generator CLI")
console = Console()

def execute_build(code: str, seed: int) -> str:
    """Executes the build(seed) function from the provided code."""
    locs = {}
    # We use a shared dict for globals and locals to behave like a module
    exec(code, locs, locs)
    if "build" not in locs:
        raise ValueError("Function 'build(seed)' not defined in the script.")
    return locs["build"](seed)

@app.command()
def generate(
    prompt: str = typer.Option(..., "--prompt", "-p", help="The description of the scenario to generate"),
    count: int = typer.Option(1, "--count", "-c", help="Number of variations to generate"),
    output_dir: str = typer.Option("datasets/benchmarks", "--output-dir", "-o", help="Root directory for generated datasets"),
    tier: str = typer.Option("spatial", "--tier", "-t", help="Tier of the scenario (e.g., spatial, kinematic)"),
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
        console.print("[bold red]Error:[/bold red] Agent failed to produce a valid script.")
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

    console.print(f"[bold green]Template generated successfully![/bold green] (ID: {scenario_id})")
    console.print(f"Entering Mass Production phase for {count} variations...")

    valid_variations = 0
    attempts = 0
    max_attempts = count * 3 # Allow some headroom for failures

    with Progress() as progress:
        task = progress.add_task("[cyan]Producing variations...", total=count)
        
        while valid_variations < count and attempts < max_attempts:
            attempts += 1
            seed = random.randint(0, 1000000)
            
            try:
                # 2. Batch Processing & Randomization
                mjcf_xml = execute_build(template_code, seed)
                
                # 3. Validation
                report = validate_mjcf(mjcf_xml)
                
                if report["is_valid"]:
                    # 4. Artifact Export
                    variation_id = f"var_{seed}"
                    
                                        # Save MJCF
                    
                                        xml_path = os.path.join(scenario_dir, f"scene_{seed}.xml")
                    
                                        with open(xml_path, "w") as f:
                    
                                            f.write(mjcf_xml)
                    
                                        
                    
                                        # Render images
                    
                                        image_prefix = os.path.join(scenario_dir, "images", f"preview_{seed}")
                    
                                        image_paths = render_scenario(mjcf_xml, image_prefix)
                    
                                        # Convert absolute paths back to relative for manifest
                    
                                        rel_image_paths = [os.path.relpath(p, scenario_dir) for p in image_paths]
                    
                                        
                    
                                        # Mock STL saving
                    
                     (since build123d normally exports them, 
                    # but our MJCF might already contain mesh references)
                    # For now we follow the requirement: Save assets/mesh_{seed}.stl
                    stl_path = os.path.join(scenario_dir, "assets", f"mesh_{seed}.stl")
                    with open(stl_path, "w") as f:
                        f.write("MOCK STL DATA") # Placeholder for now as build function returns XML
                    
                    # Save Manifest
                    manifest: ScenarioManifest = {
                        "id": f"{scenario_id}_{seed}",
                        "tier": tier,
                        "description": prompt,
                        "script_path": "template.py",
                        "assets": {
                            "mjcf": f"scene_{seed}.xml",
                            "meshes": [f"assets/mesh_{seed}.stl"],
                            "images": rel_image_paths
                        },
                        "randomization": {
                            "seed_range": [0, 1000000],
                            "parameters": ["seed"]
                        },
                        "validation": {
                            "passed": True,
                            "max_velocity": 0.0 # Should extract from report if available
                        }
                    }
                    
                    manifest_path = os.path.join(scenario_dir, f"manifest_{seed}.json")
                    with open(manifest_path, "w") as f:
                        json.dump(manifest, f, indent=2)
                    
                    valid_variations += 1
                    progress.update(task, advance=1)
                else:
                    console.print(f"[yellow]Variation with seed {seed} failed validation: {report['error_message']}[/yellow]")
            except Exception as e:
                console.print(f"[red]Error producing variation with seed {seed}: {e}[/red]")

    if valid_variations == count:
        console.print(f"\n[bold green]Success![/bold green] Generated {count} variations in {scenario_dir}")
    else:
        console.print(f"\n[bold yellow]Completed with warnings:[/bold yellow] Only generated {valid_variations}/{count} variations.")

if __name__ == "__main__":
    app()
