import os
import subprocess
import sys
from src.rag import search as rag_search

# Define a workspace directory for the agent's files
WORKSPACE_DIR = os.path.abspath("workspace")
os.makedirs(WORKSPACE_DIR, exist_ok=True)


def set_workspace_dir(path: str):
    global WORKSPACE_DIR
    WORKSPACE_DIR = os.path.abspath(path)
    os.makedirs(WORKSPACE_DIR, exist_ok=True)


def write_script(content: str, filename: str = "design.py") -> str:
    """
    Writes content to a file in the workspace.
    """
    # Ensure filename is just a name, not a path that escapes the workspace
    filename = os.path.basename(filename)
    path = os.path.join(WORKSPACE_DIR, filename)

    try:
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)
        return f"Successfully wrote to {filename}"
    except Exception as e:
        return f"Error writing to {filename}: {str(e)}"


def edit_script(filename: str, find: str, replace: str) -> str:
    """
    Replaces a unique 'find' string with 'replace' string in the specified file.
    """
    filename = os.path.basename(filename)
    path = os.path.join(WORKSPACE_DIR, filename)

    if not os.path.exists(path):
        return f"Error: File {filename} does not exist."

    try:
        with open(path, "r", encoding="utf-8") as f:
            content = f.read()

        count = content.count(find)
        if count == 0:
            return f"Error: 'find' string not found in {filename}."
        if count > 1:
            return f"Error: 'find' string is ambiguous (found {count} occurrences) in {filename}."

        new_content = content.replace(find, replace)
        with open(path, "w", encoding="utf-8") as f:
            f.write(new_content)

        return f"Successfully edited {filename}."
    except Exception as e:
        return f"Error editing {filename}: {str(e)}"


def preview_design(filename: str = "design.py") -> str:
    """
    Executes the script and renders the resulting 3D object to an image.
    Returns the path to the generated PNG.
    """
    filename = os.path.basename(filename)
    script_path = os.path.join(WORKSPACE_DIR, filename)
    output_path = os.path.join(WORKSPACE_DIR, f"{os.path.splitext(filename)[0]}.png")

    if not os.path.exists(script_path):
        return f"Error: File {filename} does not exist."

    # For MVP, we will use build123d's export_svg as a fallback if real rendering is hard
    # But let's try to implement a simple capture if possible.
    # Since we don't have a full CAD engine running here, we'll simulate the execution.

    runner_script = f"""
import os
import sys
# WARNING: This script uses exec() which is unsafe for untrusted code.
# In a production environment, this should be run in a sandboxed container.

import build123d as bd
from build123d import ExportSVG, Drawing, Unit

# Add workspace to path
sys.path.append("{WORKSPACE_DIR}")

# Execution context
locs = {{}}
try:
    with open("{script_path}", "r") as f:
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
        svg_filename = "{os.path.splitext(os.path.basename(filename))[0]}.svg"
        svg_path = os.path.join("{WORKSPACE_DIR}", svg_filename)
        
        try:
            # Create a drawing (projection) for 3D shapes
            drawing = Drawing(export_obj)
            
            exporter = ExportSVG(unit=Unit.MM)
            exporter.add_layer("visible", line_color=(0,0,0), line_weight=0.2)
            exporter.add_shape(drawing.visible_lines, layer="visible")
            # exporter.add_layer("hidden", line_color=(100,100,100), line_type=bd.LineType.DASHED)
            # exporter.add_shape(drawing.hidden_lines, layer="hidden")
            
            exporter.write(svg_path)
            print(f"RENDER_SUCCESS:{{svg_filename}}")
        except Exception as e:
            # Fallback for 2D shapes that might not work with Drawing(project) directly?
            # Drawing() should handle Shape, so it should be fine.
            print(f"RENDER_EXCEPTION:{{str(e)}}")

    else:
        print("RENDER_ERROR: No 3D object found in script (Solid, Compound, or Shape)")
except Exception as e:
    print(f"RENDER_EXCEPTION:{{str(e)}}")
"""

    try:
        # Run the runner in a separate process to avoid crashing the main process
        result = subprocess.run(
            [sys.executable, "-c", runner_script],
            capture_output=True,
            text=True,
            timeout=30,
        )

        if "RENDER_SUCCESS" in result.stdout:
            # Extract filename from stdout if needed
            try:
                # Stdout format: ... RENDER_SUCCESS:filename.svg ...
                output_line = [
                    line
                    for line in result.stdout.split("\n")
                    if "RENDER_SUCCESS" in line
                ][0]
                generated_file = output_line.split("RENDER_SUCCESS:")[1].strip()
                return f"Preview generated: {generated_file}"
            except IndexError:
                # Fallback if parsing fails but success code found (shouldn't happen)
                return f"Preview generated: {os.path.basename(output_path)}"
        else:
            error = result.stdout + result.stderr
            return f"Error generating preview: {error}"

    except subprocess.TimeoutExpired:
        return "Error: Preview generation timed out."
    except Exception as e:
        return f"Error: {str(e)}"


def search_docs(query: str) -> str:
    """
    Searches documentation (RAG).
    """
    return rag_search.search(query)
