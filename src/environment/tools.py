import functools
import hashlib
import subprocess
import sys
from pathlib import Path
from typing import Optional

import build123d as bd

from src.agent.utils.linter import format_linter_report, run_linter
from src.cots.core import PartIndex
from src.cots.providers.bd_warehouse import BDWarehouseProvider
from src.environment.sandbox import PodmanSandbox
from src.rag import search as rag_search
from src.workbenches.cnc import CNCWorkbench
from src.workbenches.injection_molding import InjectionMoldingWorkbench
from src.workbenches.print_3d import Print3DWorkbench

# Define a workspace directory for the agent's files
WORKSPACE_DIR = str(Path("workspace").resolve())
Path(WORKSPACE_DIR).mkdir(parents=True, exist_ok=True)

# Initialize Sandbox
_SANDBOX = PodmanSandbox(WORKSPACE_DIR)

# Global session tracker
_ACTIVE_SESSION_ID: Optional[str] = None


def start_session(session_id: str = "agent-session") -> str:
    """
    Starts a persistent sandbox session.
    """
    global _ACTIVE_SESSION_ID
    if _SANDBOX.start_session(session_id):
        _ACTIVE_SESSION_ID = session_id
        return f"Session {session_id} started."
    return f"Failed to start session {session_id}."


def stop_session() -> str:
    """
    Stops the current active sandbox session.
    """
    global _ACTIVE_SESSION_ID
    if _ACTIVE_SESSION_ID:
        if _SANDBOX.stop_session(_ACTIVE_SESSION_ID):
            sid = _ACTIVE_SESSION_ID
            _ACTIVE_SESSION_ID = None
            return f"Session {sid} stopped."
    return "No active session to stop."


# Initialize COTS Part Index (Singleton)
_PART_INDEX = PartIndex()
_PART_INDEX.register_provider("bd_warehouse", BDWarehouseProvider())


def set_workspace_dir(path: str):
    global WORKSPACE_DIR, _SANDBOX
    WORKSPACE_DIR = str(Path(path).resolve())
    Path(WORKSPACE_DIR).mkdir(parents=True, exist_ok=True)
    _SANDBOX = PodmanSandbox(WORKSPACE_DIR)


def write_file(content: str, path: str, mode: str = "overwrite") -> str:
    """
    Writes content to a file in the workspace or appends to it.
    If path is 'journal.md', automatically adds timestamps for append mode.
    """
    # Ensure filename is just a name or a safe relative path
    # For now, let's keep it simple and just use the name for safety unless we want subdirs
    # But user might want to write to "journal.md" or "design.py"
    full_path = Path(WORKSPACE_DIR) / path

    # Basic security check: ensure the path is within the workspace
    if not str(full_path.resolve()).startswith(str(Path(WORKSPACE_DIR).resolve())):
        return f"Error: Path {path} is outside the workspace."

    try:
        if mode == "append":
            if path == "journal.md":
                from datetime import datetime

                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                content = f"\n\n## [{timestamp}]\n{content}\n"

            with full_path.open("a", encoding="utf-8") as f:
                f.write(content)
            result = f"Successfully appended to {path}"
        else:
            full_path.parent.mkdir(parents=True, exist_ok=True)
            full_path.write_text(content, encoding="utf-8")
            result = f"Successfully wrote to {path}"

        if path.endswith(".py"):
            lint_report = lint_script(path)
            result += f"\n\n{lint_report}"
        return result
    except Exception as e:
        return f"Error writing to {path}: {e!s}"


def edit_file(path: str, find: str, replace: str) -> str:
    """
    Replaces a unique 'find' string with 'replace' string in the specified file.
    """
    full_path = Path(WORKSPACE_DIR) / path

    if not full_path.exists():
        return f"Error: File {path} does not exist."

    try:
        content = full_path.read_text(encoding="utf-8")

        count = content.count(find)
        if count == 0:
            return f"Error: 'find' string not found in {path}."
        if count > 1:
            return f"Error: 'find' string is ambiguous (found {count} occurrences) in {path}."

        new_content = content.replace(find, replace)
        full_path.write_text(new_content, encoding="utf-8")

        result = f"Successfully edited {path}."
        if path.endswith(".py"):
            lint_report = lint_script(path)
            result += f"\n\n{lint_report}"
        return result
    except Exception as e:
        return f"Error editing {path}: {e!s}"


def write_script(content: str, filename: str = "design.py") -> str:
    """
    [DEPRECATED] Writes content to a file in the workspace. Use write_file instead.
    """
    return write_file(content, filename, mode="overwrite")


def edit_script(filename: str, find: str, replace: str) -> str:
    """
    [DEPRECATED] Replaces a unique 'find' string with 'replace' string. Use edit_file instead.
    """
    return edit_file(filename, find, replace)


def read_script(filename: str = "design.py") -> str:
    """
    Reads the content of a script from the workspace.
    """
    return view_file(filename)


def view_file(path: str) -> str:
    """
    Reads the content of any file in the workspace or mounted docs.
    """
    # If it's a relative path starting with docs/skills, we look in the host skills dir
    # because the agent might try to read it before a session is started,
    # OR it's mounted in the container.

    if path.startswith("docs/skills/"):
        # Map back to host path .agent/skills/...
        skill_rel = path.replace("docs/skills/", "")
        host_path = Path(".agent/skills") / skill_rel
        if host_path.exists():
            try:
                return host_path.read_text(encoding="utf-8")
            except Exception as e:
                return f"Error reading skill file: {e!s}"

    filename = Path(path).name
    full_path = Path(WORKSPACE_DIR) / path  # Allow subdirs in workspace if they exist

    if not full_path.exists():
        # Check if it's just the filename in the workspace (backward compatibility)
        full_path = Path(WORKSPACE_DIR) / filename

    if not full_path.exists():
        return f"Error: File {path} does not exist."

    try:
        return full_path.read_text(encoding="utf-8")
    except Exception as e:
        return f"Error reading {path}: {e!s}"


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
        stdout, stderr, returncode = _SANDBOX.run_script(
            runner_filename, session_id=_ACTIVE_SESSION_ID
        )

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


def read_skill(
    skill_name: str, filename: str = "SKILL.md", resource_type: Optional[str] = None
) -> str:
    """
    Reads information from a specialized skill folder.
    resource_type can be 'scripts', 'references', or 'assets'.
    If not provided, it tries root, then references, then scripts, then assets.
    """
    skills_dir = Path(".agent/skills") / skill_name
    if not skills_dir.exists():
        return f"Error: Skill '{skill_name}' does not exist."

    # Validate filename to prevent path traversal
    filename = Path(filename).name

    if resource_type:
        target_path = skills_dir / resource_type / filename
    else:
        # Priority search
        target_path = skills_dir / filename
        if not target_path.exists():
            target_path = skills_dir / "references" / filename
        if not target_path.exists():
            target_path = skills_dir / "scripts" / filename
        if not target_path.exists():
            target_path = skills_dir / "assets" / filename

    if not target_path.exists():
        return f"Error: File '{filename}' not found in skill '{skill_name}'."

    try:
        if _ACTIVE_SESSION_ID:
            # If session is active, we can just cat the file from the container mount
            # But reading from host is often faster and always works if mounted.
            pass
        return target_path.read_text(encoding="utf-8")
    except Exception as e:
        return f"Error reading skill: {e!s}"


def run_command(command: str, timeout: int = 60) -> str:
    """
    Runs a shell command in the persistent sandbox.
    If no session is active, it runs it in a transient one (slower).
    """
    import shlex

    cmd_args = shlex.split(command)

    if _ACTIVE_SESSION_ID:
        stdout, stderr, returncode = _SANDBOX.exec_command(
            _ACTIVE_SESSION_ID, cmd_args, timeout=timeout
        )
    else:
        # Fallback to transient run
        # We need to construct a similar environment
        stdout, stderr, returncode = _SANDBOX.run_script(
            "-c", timeout=timeout
        )  # This won't work well for raw commands
        # Let's actually enforce session for run_command in the agent prompts
        return "Error: No active sandbox session. Please start a session before running commands."

    result = stdout
    if stderr:
        result += f"\nSTDERR:\n{stderr}"
    if returncode != 0:
        result += f"\nReturn Code: {returncode}"
    return result


def run_skill_script(skill_name: str, script_name: str, arguments: str = "") -> str:
    """
    Executes a script from a specialized skill's 'scripts' folder.
    """
    script_path = Path(".agent/skills") / skill_name / "scripts" / script_name
    if not script_path.exists():
        return f"Error: Script '{script_name}' not found in skill '{skill_name}'."

    try:
        # Determine execution command
        if script_path.suffix == ".py":
            cmd = [sys.executable, str(script_path)]
        elif script_path.suffix == ".sh":
            cmd = ["bash", str(script_path)]
        else:
            return f"Error: Unsupported script type '{script_path.suffix}'"

        if arguments:
            import shlex

            cmd.extend(shlex.split(arguments))

        result = subprocess.run(cmd, capture_output=True, text=True, check=False)
        output = result.stdout
        if result.stderr:
            output += f"\nSTDERR:\n{result.stderr}"

        return output if output else "Script executed successfully (no output)."
    except Exception as e:
        return f"Error executing skill script: {e!s}"


def list_skills() -> str:
    """
    Lists all available specialized skills.
    """
    skills_dir = Path(".agent/skills")
    if not skills_dir.exists():
        return "No skills found."

    skills = sorted([p.name for p in skills_dir.iterdir() if p.is_dir()])
    if not skills:
        return "No skills found."

    return "Available skills:\n- " + "\n- ".join(skills)


def list_skill_files(skill_name: str) -> str:
    """
    Lists all files within a specialized skill folder, grouped by type.
    """
    skills_dir = Path(".agent/skills") / skill_name
    if not skills_dir.exists():
        return f"Error: Skill '{skill_name}' does not exist."

    result = [f"Contents of skill '{skill_name}':"]

    if (skills_dir / "SKILL.md").exists():
        result.append("- SKILL.md")

    for folder in ["scripts", "references", "assets"]:
        subdir = skills_dir / folder
        if subdir.exists() and subdir.is_dir():
            files = sorted(
                [str(p.relative_to(subdir)) for p in subdir.rglob("*") if p.is_file()]
            )
            if files:
                result.append(f"- {folder}/")
                for f in files:
                    result.append(f"  - {f}")

    return "\n".join(result)


def update_skill(
    skill_name: str,
    content: str,
    filename: str = "SKILL.md",
    resource_type: Optional[str] = None,
) -> str:
    """
    Updates or adds information to a specialized skill folder.
    resource_type can be 'scripts', 'references', or 'assets'.
    If filename is 'SKILL.md', it goes to root.
    If filename is not 'SKILL.md' and resource_type is not provided, it defaults to 'references'.
    """
    skills_dir = Path(".agent/skills") / skill_name
    if not skills_dir.exists():
        return f"Error: Skill '{skill_name}' does not exist. Use init_skill first."

    # Validate filename to prevent path traversal
    filename = Path(filename).name

    # Validation logic
    rtype = resource_type
    if filename == "SKILL.md":
        target_path = skills_dir / filename
        rtype = None
    else:
        rtype = resource_type or "references"
        target_path = skills_dir / rtype / filename
        target_path.parent.mkdir(parents=True, exist_ok=True)

    # Check extensions
    if rtype == "references" and not filename.endswith(".md"):
        return "Error: Only .md files can be added to references."
    if rtype == "scripts" and not (
        filename.endswith(".py") or filename.endswith(".sh")
    ):
        return "Error: Only .py or .sh files can be added to scripts."

    try:
        target_path.write_text(content, encoding="utf-8")
        if target_path.suffix in [".py", ".sh"] or rtype == "scripts":
            target_path.chmod(0o755)
        rel_path = target_path.relative_to(skills_dir)
        return f"Successfully updated skill '{skill_name}' at {rel_path}"
    except Exception as e:
        return f"Error updating skill: {e!s}"


def init_skill(skill_name: str) -> str:
    """
    Initializes a new skill using the canonical init_skill.py script.
    """
    init_script = Path(".agent/skills/skill-creator/scripts/init_skill.py")
    if not init_script.exists():
        return "Error: init_skill.py not found in skill-creator."

    try:
        cmd = [sys.executable, str(init_script), skill_name, "--path", ".agent/skills"]
        result = subprocess.run(cmd, capture_output=True, text=True, check=False)
        if result.returncode == 0:
            return f"Successfully initialized skill '{skill_name}'.\n{result.stdout}"
        else:
            return f"Error initializing skill: {result.stderr or result.stdout}"
    except Exception as e:
        return f"Exception initializing skill: {e!s}"


def package_skill(skill_name: str) -> str:
    """
    Validates and packages a skill using the canonical package_skill.py script.
    """
    package_script = Path(".agent/skills/skill-creator/scripts/package_skill.py")
    if not package_script.exists():
        return "Error: package_skill.py not found in skill-creator."

    skill_path = Path(".agent/skills") / skill_name
    if not skill_path.exists():
        return f"Error: Skill path '{skill_path}' does not exist."

    try:
        cmd = [sys.executable, str(package_script), str(skill_path)]
        result = subprocess.run(cmd, capture_output=True, text=True, check=False)
        if result.returncode == 0:
            return f"Successfully packaged skill '{skill_name}'.\n{result.stdout}"
        else:
            return f"Error packaging skill (validation failed):\n{result.stderr or result.stdout}"
    except Exception as e:
        return f"Exception packaging skill: {e!s}"


# Global workbenches to avoid re-initializing config
_WORKBENCHES = {
    "cnc": CNCWorkbench(),
    "cnc_milling": CNCWorkbench(),
    "injection_molding": InjectionMoldingWorkbench(),
    "print_3d": Print3DWorkbench(),
}


@functools.lru_cache(maxsize=128)
def _analyze_cached(
    file_hash: str, default_process: str, default_quantity: int, script_content: str
):
    """Internal helper to cache analysis results based on file content hash."""
    # Execute the script to get the part
    locs = {}
    exec(script_content, globals(), locs)

    # Collect all exportable objects
    exportable_objects = []
    for val in locs.values():
        if isinstance(val, (bd.Compound, bd.Solid, bd.Shape)):
            exportable_objects.append(val)
        elif isinstance(val, list):
            for item in val:
                if isinstance(item, (bd.Compound, bd.Solid, bd.Shape)):
                    exportable_objects.append(item)
        elif hasattr(val, "part") and isinstance(val.part, bd.Shape):
            exportable_objects.append(val.part)

    if not exportable_objects:
        raise ValueError("No 3D object found in script")

    # Helper to parse label: "quantity:100|process:cnc"
    def parse_label(label: str):
        data = {"quantity": default_quantity, "process": default_process}
        if not label:
            return data
        parts = str(label).split("|")
        for p in parts:
            if ":" in p:
                k, v = p.split(":", 1)
                if k == "quantity":
                    try:
                        data["quantity"] = int(v)
                    except ValueError:
                        pass
                elif k == "process":
                    data["process"] = v
        return data

    all_solids_raw = []
    for obj in exportable_objects:
        if isinstance(obj, bd.Compound):
            all_solids_raw.extend(list(obj.solids()))
        elif isinstance(obj, bd.Solid):
            all_solids_raw.append(obj)
        else:
            try:
                all_solids_raw.extend(list(obj.solids()))
            except:
                pass

    # Deduplicate by geometric hash
    all_solids = []
    seen_hashes = set()
    seen_ids = set()  # Fallback for non-solid shapes if any
    for s in all_solids_raw:
        # Create a simple hash based on center and volume
        try:
            c = s.center()
            geom_hash = hashlib.md5(
                f"{c.X:.4f},{c.Y:.4f},{c.Z:.4f},{s.volume:.4f}".encode()
            ).hexdigest()
            if geom_hash not in seen_hashes:
                all_solids.append(s)
                seen_hashes.add(geom_hash)
        except:
            # If center() or volume fails, fallback to ID just in case
            if id(s) not in seen_ids:
                all_solids.append(s)
                seen_ids.add(id(s))

    total_cost = 0.0
    all_violations = []
    reuse_context = {}  # Tracks part hashes for reuse discounts
    part_reports = []

    for i, solid in enumerate(all_solids):
        label_data = parse_label(getattr(solid, "label", ""))
        process = label_data["process"]
        quantity = label_data["quantity"]

        workbench = _WORKBENCHES.get(process)
        if not workbench:
            # Fallback to default if labeled process is unknown
            workbench = _WORKBENCHES.get(default_process)
            process = default_process

        violations = workbench.validate(solid)
        all_violations.extend(violations)

        cost = workbench.calculate_cost(solid, quantity, context=reuse_context)
        total_cost += cost

        part_reports.append(
            {
                "part_index": i,
                "process": process,
                "quantity": quantity,
                "cost": cost,
                "violations": violations,
            }
        )

    status = "pass" if not all_violations else "fail"

    mapped_violations = []
    for v in all_violations:
        mapped_violations.append(
            {"type": "generic_violation", "description": str(v), "severity": "critical"}
        )

    report = {
        "status": status,
        "manufacturability_score": 1.0
        if status == "pass"
        else max(0.0, 1.0 - len(all_violations) * 0.1),
        "violations": mapped_violations,
        "parts": part_reports,
        "cost_analysis": {
            "total_cost": total_cost,
            "unit_cost": total_cost / default_quantity if default_quantity > 0 else 0.0,
            "target_quantity": default_quantity,
        },
    }
    return report


def check_manufacturability(
    design_file: str = "design.py", process: str = "cnc", quantity: int = 1
) -> dict:
    """
    Checks if the design in the specified file can be manufactured using the target process.
    Supported processes: 'cnc', 'injection_molding'.
    """
    design_file = Path(design_file).name
    script_path = Path(WORKSPACE_DIR) / design_file

    if not script_path.exists():
        return {"error": f"File {design_file} does not exist."}

    try:
        # Compute hash of file content for caching
        # Compute hash of file content for caching
        with script_path.open("r") as f:
            script_content = f.read()
        file_hash = hashlib.md5(script_content.encode()).hexdigest()

        return _analyze_cached(file_hash, process, quantity, script_content)
    except Exception as e:
        return {"error": str(e)}


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


def lint_script(filename: str = "design.py") -> str:
    """
    Runs static analysis (Ruff, Pyrefly) on a script in the workspace.
    Returns a formatted report of errors and suggestions.
    """
    filename = Path(filename).name
    path = Path(WORKSPACE_DIR) / filename

    if not path.exists():
        return f"Error: File {filename} does not exist."

    try:
        code = path.read_text(encoding="utf-8")
        errors = run_linter(code)
        if not errors:
            return "No linting errors found. Code looks clean!"
        return format_linter_report(errors)
    except Exception as e:
        return f"Error running linter: {e!s}"
