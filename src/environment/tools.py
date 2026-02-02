from pathlib import Path

from src.cots.core import PartIndex
from src.cots.providers.bd_warehouse import BDWarehouseProvider
from src.environment.sandbox import PodmanSandbox
from src.rag import search as rag_search

# Define a workspace directory for the agent's files
WORKSPACE_DIR = str(Path("workspace").resolve())
Path(WORKSPACE_DIR).mkdir(parents=True, exist_ok=True)

# Initialize Sandbox
_SANDBOX = PodmanSandbox(WORKSPACE_DIR)

# Initialize COTS Part Index (Singleton)
_PART_INDEX = PartIndex()
_PART_INDEX.register_provider("bd_warehouse", BDWarehouseProvider())


def set_workspace_dir(path: str):
    global WORKSPACE_DIR, _SANDBOX
    WORKSPACE_DIR = str(Path(path).resolve())
    Path(WORKSPACE_DIR).mkdir(parents=True, exist_ok=True)
    _SANDBOX = PodmanSandbox(WORKSPACE_DIR)


def write_script(content: str, filename: str = "design.py") -> str:
    """
    Writes content to a file in the workspace.
    """
    # Ensure filename is just a name, not a path that escapes the workspace
    filename = Path(filename).name
    path = Path(WORKSPACE_DIR) / filename

    try:
        path.write_text(content, encoding="utf-8")
        return f"Successfully wrote to {filename}"
    except Exception as e:
        return f"Error writing to {filename}: {e!s}"


def edit_script(filename: str, find: str, replace: str) -> str:
    """
    Replaces a unique 'find' string with 'replace' string in the specified file.
    """
    filename = Path(filename).name
    path = Path(WORKSPACE_DIR) / filename

    if not path.exists():
        return f"Error: File {filename} does not exist."

    try:
        content = path.read_text(encoding="utf-8")

        count = content.count(find)
        if count == 0:
            return f"Error: 'find' string not found in {filename}."
        if count > 1:
            return f"Error: 'find' string is ambiguous (found {count} occurrences) in {filename}."

        new_content = content.replace(find, replace)
        path.write_text(new_content, encoding="utf-8")

        return f"Successfully edited {filename}."
    except Exception as e:
        return f"Error editing {filename}: {e!s}"


def preview_design(filename: str = "design.py") -> str:
    """
    Executes the script and renders the resulting 3D object to an image.
    Returns the path to the generated PNG.
    """
    filename = Path(filename).name
    script_path = Path(WORKSPACE_DIR) / filename

    if not script_path.exists():
        return f"Error: File {filename} does not exist."

    # Use PodmanSandbox for execution
    # We create a temporary runner script in the workspace
    runner_filename = f"runner_{filename}"
    runner_path = Path(WORKSPACE_DIR) / runner_filename

    runner_script = f"""
import sys
from pathlib import Path
import build123d as bd
from build123d import ExportSVG, Drawing, Unit

# Add workspace to path
sys.path.append("/workspace")

# Execution context
locs = {{}}
try:
    with Path("/workspace/{filename}").open("r") as f:
        code = f.read()
    exec(code, globals(), locs)
    
    # Try to find a compound or part
    export_obj = None
    for val in locs.values():
        if isinstance(val, (bd.Compound, bd.Solid, bd.Shape)):
            export_obj = val
            break
        elif hasattr(val, "part") and isinstance(val.part, bd.Shape):
            export_obj = val.part
            break
        elif hasattr(val, "sketch") and isinstance(val.sketch, bd.Shape):
            export_obj = val.sketch
            break
        elif hasattr(val, "line") and isinstance(val.line, bd.Shape):
            export_obj = val.line
            break
            
    if export_obj:
        # Export as SVG using built-in exporters
        svg_filename = f"{{Path('{filename}').stem}}.svg"
        svg_path = Path("/workspace") / svg_filename
        
        try:
            # Create a drawing (projection) for 3D shapes
            drawing = Drawing(export_obj)
            
            exporter = ExportSVG(unit=Unit.MM)
            exporter.add_layer("visible", line_color=(0,0,0), line_weight=0.2)
            exporter.add_shape(drawing.visible_lines, layer="visible")
            
            exporter.write(str(svg_path))
            print(f"RENDER_SUCCESS:{{svg_filename}}")
        except Exception as e:
            print(f"RENDER_EXCEPTION:{{str(e)}}")
    else:
        print("RENDER_ERROR: No 3D object found in script (Solid, Compound, or Shape)")
except Exception as e:
    print(f"RENDER_EXCEPTION:{{str(e)}}")
"""

    try:
        # Write runner script
        runner_path.write_text(runner_script, encoding="utf-8")

        # Run in sandbox
        stdout, stderr, returncode = _SANDBOX.run_script(runner_filename)

        # Clean up runner script
        if runner_path.exists():
            runner_path.unlink()

        if "RENDER_SUCCESS" in stdout:
            try:
                output_line = [
                    line for line in stdout.split("\n") if "RENDER_SUCCESS" in line
                ][0]
                generated_file = output_line.split("RENDER_SUCCESS:")[1].strip()
                return f"Preview generated: {generated_file}"
            except IndexError:
                return "Preview generated (Filename unknown)"
        else:
            error = stdout + stderr
            return f"Error generating preview: {error}"

    except Exception as e:
        return f"Error: {e!s}"


def search_docs(query: str) -> str:
    """
    Searches documentation (RAG).
    """
    return rag_search.search(query)


def search_parts(query: str) -> str:
    """
    Search for COTS parts by name or ID. Returns a list of matches.
    """
    try:
        results = _PART_INDEX.search(query)
        if not results:
            return f"No parts found for query: {query}"

        output = "Found parts:\n"
        for res in results:
            output += f"- {res.id}: {res.name} (Provider: {res.provider})\n"
        return output
    except Exception as e:
        return f"Error searching parts: {e!s}"


def preview_part(part_id: str) -> str:
    """
    Get visual preview and details for a part ID.
    Returns description, image path, and a Python recipe to instantiate it.
    """
    try:
        preview = _PART_INDEX.preview(part_id)
        output = f"Part: {preview.id}\n"
        output += f"Description: {preview.description}\n"
        output += f"Image: {preview.image_path}\n"
        output += f"Recipe:\n```python\n{preview.recipe}\n```\n"
        if preview.metadata:
            output += f"Metadata: {preview.metadata}\n"
        return output
    except Exception as e:
        return f"Error previewing part {part_id}: {e!s}"
