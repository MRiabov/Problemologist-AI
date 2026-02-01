import asyncio
from src.environment import tools as env_tools

async def write_script_async(content: str, path: str) -> str:
    """Async wrapper for writing a script."""
    # Run synchronous IO in a thread pool to avoid blocking the event loop
    return await asyncio.to_thread(env_tools.write_script, content, path)

async def edit_script_async(path: str, find: str, replace: str) -> str:
    """Async wrapper for editing a script."""
    return await asyncio.to_thread(env_tools.edit_script, path, find, replace)

async def preview_design_async(path: str) -> str:
    """Async wrapper for previewing a design."""
    return await asyncio.to_thread(env_tools.preview_design, path)

async def search_docs_async(query: str) -> str:
    """Async wrapper for searching documentation."""
    return await asyncio.to_thread(env_tools.search_docs, query)

# submit_design is not in src.environment.tools yet, we'll keep it as a placeholder or mock
async def submit_design_async(control_path: str) -> str:
    """Async wrapper for submitting a design."""
    # Placeholder for future MuJoCo integration
    await asyncio.sleep(1) # Simulate some work
    return "Design submitted for validation. (Simulation results would appear here)"
