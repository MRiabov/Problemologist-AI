import asyncio
import typer
from pathlib import Path
import structlog

# Adjust import path based on where this file is located relative to src
# If running as module python -m shared.cli.benchmark, this works.
from worker.generators.benchmark.graph import run_generation_session
from worker.generators.benchmark.models import SessionStatus

app = typer.Typer(help="Benchmark Generation CLI")
logger = structlog.get_logger()

async def _run_generate(prompt: str):
    print(f"Starting session for prompt: '{prompt}'")
    try:
        final_state = await run_generation_session(prompt)
        session = final_state["session"]
        
        print(f"Session ID: {session.session_id}")
        print(f"Status: {session.status}")
        
        if session.status == SessionStatus.accepted:
            print("Asset generated successfully.")
        else:
            print("Generation failed or rejected.")
            
    except Exception as e:
        print(f"Error: {e}")
        logger.error("cli_error", error=str(e))

@app.command()
def generate(prompt: str):
    """Generate a single benchmark scenario."""
    asyncio.run(_run_generate(prompt))

@app.command()
def batch(file: Path):
    """Generate benchmarks from a file (one prompt per line)."""
    if not file.exists():
        typer.echo(f"File {file} not found.", err=True)
        raise typer.Exit(code=1)
    
    prompts = file.read_text().splitlines()
    # Run sequentially for now
    for p in prompts:
        p = p.strip()
        if p and not p.startswith("#"):
            asyncio.run(_run_generate(p))

if __name__ == "__main__":
    app()
