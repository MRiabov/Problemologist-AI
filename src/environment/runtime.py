import functools
import hashlib
import json
import asyncio
import subprocess
import sys
from pathlib import Path
from typing import Optional, Any, Dict, List, Tuple

import build123d as bd

from src.agent.utils.linter import format_linter_report, run_linter
from src.cots.core import PartIndex
from src.cots.providers.bd_warehouse import BDWarehouseProvider
from src.environment.sandbox import PodmanSandbox
from src.rag import search as rag_search
from src.workbenches.cnc import CNCWorkbench
from src.workbenches.injection_molding import InjectionMoldingWorkbench
from src.workbenches.print_3d import Print3DWorkbench


class ToolRuntime:
    """
    Encapsulates the runtime state and tool implementations for the CAD environment.
    Replaces global state from the legacy tools.py module.
    """

    def __init__(self, workspace_dir: str, db: Any = None):
        self.workspace_dir = str(Path(workspace_dir).resolve())
        Path(self.workspace_dir).mkdir(parents=True, exist_ok=True)

        self.sandbox = PodmanSandbox(self.workspace_dir)
        self.db = db

        self.active_session_id: Optional[str] = None

        # Initialize COTS Part Index
        self.part_index = PartIndex()
        self.part_index.register_provider("bd_warehouse", BDWarehouseProvider())

        # Workbenches
        self.workbenches = {
            "cnc": CNCWorkbench(),
            "cnc_milling": CNCWorkbench(),
            "injection_molding": InjectionMoldingWorkbench(),
            "print_3d": Print3DWorkbench(),
        }

    def start_session(self, session_id: str = "agent-session") -> str:
        """Starts a persistent sandbox session."""
        if self.sandbox.start_session(session_id):
            self.active_session_id = session_id
            return f"Session {session_id} started."
        return f"Failed to start session {session_id}."

    def stop_session(self) -> str:
        """Stops the current active sandbox session."""
        if self.active_session_id:
            if self.sandbox.stop_session(self.active_session_id):
                sid = self.active_session_id
                self.active_session_id = None
                return f"Session {sid} stopped."
        return "No active session to stop."

    def write_file(self, content: str, path: str, mode: str = "overwrite") -> str:
        """Writes content to a file in the workspace or appends to it."""
        full_path = Path(self.workspace_dir) / path

        if not str(full_path.resolve()).startswith(
            str(Path(self.workspace_dir).resolve())
        ):
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
                lint_report = self.lint_script(path)
                result += f"\n\n{lint_report}"
            return result
        except Exception as e:
            return f"Error writing to {path}: {e!s}"

    def edit_file(self, path: str, find: str, replace: str) -> str:
        """Replaces a unique 'find' string with 'replace' string in the specified file."""
        full_path = Path(self.workspace_dir) / path

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
                lint_report = self.lint_script(path)
                result += f"\n\n{lint_report}"
            return result
        except Exception as e:
            return f"Error editing {path}: {e!s}"

    def read_script(self, filename: str = "design.py") -> str:
        """Reads the content of a script from the workspace."""
        return self.view_file(filename)

    def view_file(self, path: str) -> str:
        """Reads the content of any file in the workspace or mounted docs."""
        if path.startswith("docs/skills/"):
            skill_rel = path.replace("docs/skills/", "")
            host_path = Path(".agent/skills") / skill_rel
            if host_path.exists():
                try:
                    return host_path.read_text(encoding="utf-8")
                except Exception as e:
                    return f"Error reading skill file: {e!s}"

        filename = Path(path).name
        full_path = Path(self.workspace_dir) / path

        if not full_path.exists():
            full_path = Path(self.workspace_dir) / filename

        if not full_path.exists():
            return f"Error: File {path} does not exist."

        try:
            return full_path.read_text(encoding="utf-8")
        except Exception as e:
            return f"Error reading {path}: {e!s}"

    def preview_design(self, filename: str = "design.py") -> str:
        """Executes the script and renders the resulting 3D object to an image."""
        filename = Path(filename).name
        script_path = Path(self.workspace_dir) / filename

        if not script_path.exists():
            return f"Error: File {filename} does not exist."

        runner_filename = f"runner_{filename}"
        runner_path = Path(self.workspace_dir) / runner_filename

        runner_script = f"""
import sys
from pathlib import Path
import build123d as bd
from build123d import ExportSVG, Drawing, Unit

sys.path.append("/workspace")

locs = {{}}
try:
    with Path("/workspace/{filename}").open("r") as f:
        code = f.read()
    exec(code, globals(), locs)
    
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
        svg_filename = f"{{Path('{filename}').stem}}.svg"
        svg_path = Path("/workspace") / svg_filename
        
        try:
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
            runner_path.write_text(runner_script, encoding="utf-8")
            stdout, stderr, returncode = self.sandbox.run_script(
                runner_filename, session_id=self.active_session_id
            )

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
        self,
        skill_name: str,
        filename: str = "SKILL.md",
        resource_type: Optional[str] = None,
    ) -> str:
        """Reads information from a specialized skill folder."""
        skills_dir = Path(".agent/skills") / skill_name
        if not skills_dir.exists():
            return f"Error: Skill '{skill_name}' does not exist."

        filename = Path(filename).name

        if resource_type:
            target_path = skills_dir / resource_type / filename
        else:
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
            return target_path.read_text(encoding="utf-8")
        except Exception as e:
            return f"Error reading skill: {e!s}"

    def run_command(self, command: str, timeout: int = 60) -> str:
        """Runs a shell command in the persistent sandbox."""
        import shlex

        cmd_args = shlex.split(command)

        if self.active_session_id:
            stdout, stderr, returncode = self.sandbox.exec_command(
                self.active_session_id, cmd_args, timeout=timeout
            )
        else:
            stdout, stderr, returncode = self.sandbox.run_script(
                "-c",
                timeout=timeout,  # This is just a place holder, run_script is not really designed for raw commands
            )
            return "Error: No active sandbox session. Please start a session before running commands."

        result = stdout
        if stderr:
            result += f"\nSTDERR:\n{stderr}"
        if returncode != 0:
            result += f"\nReturn Code: {returncode}"
        return result

    def run_skill_script(
        self, skill_name: str, script_name: str, arguments: str = ""
    ) -> str:
        """Executes a script from a specialized skill's 'scripts' folder."""
        script_path = Path(".agent/skills") / skill_name / "scripts" / script_name
        if not script_path.exists():
            return f"Error: Script '{script_name}' not found in skill '{skill_name}'."

        try:
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

    def list_skills(self) -> str:
        """Lists all available specialized skills."""
        skills_dir = Path(".agent/skills")
        if not skills_dir.exists():
            return "No skills found."

        skills = sorted([p.name for p in skills_dir.iterdir() if p.is_dir()])
        if not skills:
            return "No skills found."

        return "Available skills:\n- " + "\n- ".join(skills)

    def list_skill_files(self, skill_name: str) -> str:
        """Lists all files within a specialized skill folder, grouped by type."""
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
                    [
                        str(p.relative_to(subdir))
                        for p in subdir.rglob("*")
                        if p.is_file()
                    ]
                )
                if files:
                    result.append(f"- {folder}/")
                    for f in files:
                        result.append(f"  - {f}")

        return "\n".join(result)

    def update_skill(
        self,
        skill_name: str,
        content: str,
        filename: str = "SKILL.md",
        resource_type: Optional[str] = None,
    ) -> str:
        """Updates or adds information to a specialized skill folder."""
        skills_dir = Path(".agent/skills") / skill_name
        if not skills_dir.exists():
            return f"Error: Skill '{skill_name}' does not exist. Use init_skill first."

        filename = Path(filename).name
        rtype = resource_type
        if filename == "SKILL.md":
            target_path = skills_dir / filename
            rtype = None
        else:
            rtype = resource_type or "references"
            target_path = skills_dir / rtype / filename
            target_path.parent.mkdir(parents=True, exist_ok=True)

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

    def init_skill(self, skill_name: str) -> str:
        """Initializes a new skill using the canonical init_skill.py script."""
        init_script = Path(".agent/skills/skill-creator/scripts/init_skill.py")
        if not init_script.exists():
            return "Error: init_skill.py not found in skill-creator."

        try:
            cmd = [
                sys.executable,
                str(init_script),
                skill_name,
                "--path",
                ".agent/skills",
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, check=False)
            if result.returncode == 0:
                return (
                    f"Successfully initialized skill '{skill_name}'.\n{result.stdout}"
                )
            else:
                return f"Error initializing skill: {result.stderr or result.stdout}"
        except Exception as e:
            return f"Exception initializing skill: {e!s}"

    def package_skill(self, skill_name: str) -> str:
        """Validates and packages a skill using the canonical package_skill.py script."""
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

    def search_docs(self, query: str) -> str:
        """Searches documentation (RAG)."""
        return rag_search.search(query)

    def search_parts(self, query: str) -> str:
        """Search for COTS parts by name or ID."""
        try:
            results = self.part_index.search(query)
            if not results:
                return f"No parts found for query: {query}"

            output = "Found parts:\n"
            for res in results:
                output += f"- {res.id}: {res.name} (Provider: {res.provider})\n"
            return output
        except Exception as e:
            return f"Error searching parts: {e!s}"

    def preview_part(self, part_id: str) -> str:
        """Get visual preview and details for a part ID."""
        try:
            preview = self.part_index.preview(part_id)
            output = f"Part: {preview.id}\n"
            output += f"Description: {preview.description}\n"
            output += f"Image: {preview.image_path}\n"
            output += f"Recipe:\n```python\n{preview.recipe}\n```\n"
            if preview.metadata:
                output += f"Metadata: {preview.metadata}\n"
            return output
        except Exception as e:
            return f"Error previewing part {part_id}: {e!s}"

    def lint_script(self, filename: str = "design.py") -> str:
        """Runs static analysis on a script in the workspace."""
        filename = Path(filename).name
        path = Path(self.workspace_dir) / filename

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

    def check_manufacturability(
        self, design_file: str = "design.py", process: str = "cnc", quantity: int = 1
    ) -> Dict[str, Any]:
        """Checks if the design in the specified file can be manufactured using the target process. Safe version running in sandbox."""
        design_file = Path(design_file).name
        script_path = Path(self.workspace_dir) / design_file

        if not script_path.exists():
            return {"error": f"File {design_file} does not exist."}

        # Create a temporary runner script to execute analysis inside the sandbox
        runner_filename = (
            f"analysis_runner_{hashlib.md5(design_file.encode()).hexdigest()[:8]}.py"
        )
        runner_path = Path(self.workspace_dir) / runner_filename

        # We need to escape the script content properly when writing it to the runner
        # This runner will import the design and run validation
        runner_script = f"""
import sys
import json
import hashlib
from pathlib import Path
import build123d as bd
from src.workbenches.cnc import CNCWorkbench
from src.workbenches.injection_molding import InjectionMoldingWorkbench
from src.workbenches.print_3d import Print3DWorkbench

sys.path.append("/workspace")

# Define result structure
result = {{
    "status": "fail", 
    "manufacturability_score": 0.0,
    "violations": [],
    "parts": [],
    "cost_analysis": {{
        "total_cost": 0.0,
        "unit_cost": 0.0,
        "target_quantity": {quantity}
    }},
    "error": None
}}

try:
    # Load design
    locs = {{}}
    with Path("/workspace/{design_file}").open("r") as f:
        code = f.read()
    exec(code, globals(), locs)
    
    # Extract solids
    all_solids = []
    exportable = []
    
    for val in locs.values():
        if isinstance(val, (bd.Compound, bd.Solid, bd.Shape)):
            exportable.append(val)
        elif isinstance(val, list):
            for item in val:
                if isinstance(item, (bd.Compound, bd.Solid, bd.Shape)):
                    exportable.append(item)
        elif hasattr(val, "part") and isinstance(val.part, bd.Shape):
            exportable.append(val.part)

    for obj in exportable:
        if isinstance(obj, bd.Compound):
            all_solids.extend(list(obj.solids()))
        elif isinstance(obj, bd.Solid):
            all_solids.append(obj)
        else:
            try:
                all_solids.extend(list(obj.solids()))
            except:
                pass
                
    if not all_solids:
        raise ValueError("No 3D object found in script")

    # Analyze
    workbenches = {{
        "cnc": CNCWorkbench(),
        "injection_molding": InjectionMoldingWorkbench(),
        "print_3d": Print3DWorkbench(),
    }}
    
    target_process = "{process}"
    default_process = target_process
    
    total_cost = 0.0
    all_violations = []
    part_reports = []
    reuse_context = {{}}

    # Helper for parsing labels
    def parse_label(label):
        data = {{"quantity": {quantity}, "process": default_process}}
        if not label: return data
        for p in str(label).split("|"):
            if ":" in p:
                k, v = p.split(":", 1)
                if k == "quantity": 
                    try: data["quantity"] = int(v)
                    except: pass
                elif k == "process": data["process"] = v
        return data

    for i, solid in enumerate(all_solids):
        label_data = parse_label(getattr(solid, "label", ""))
        proc = label_data["process"]
        qty = label_data["quantity"]
        
        wb = workbenches.get(proc, workbenches.get(default_process))
        if not wb: wb = workbenches["print_3d"]
        
        violations = wb.validate(solid)
        all_violations.extend(violations)
        
        cost = wb.calculate_cost(solid, qty, context=reuse_context)
        total_cost += cost
        
        part_reports.append({{
            "part_index": i,
            "process": proc,
            "quantity": qty,
            "cost": cost,
            "violations": [str(v) for v in violations]
        }})
        
    result["status"] = "pass" if not all_violations else "fail"
    result["manufacturability_score"] = 1.0 if not all_violations else max(0.0, 1.0 - len(all_violations) * 0.1)
    
    mapped_violations = []
    for v in all_violations:
        mapped_violations.append({{
            "type": "generic_violation", 
            "description": str(v), 
            "severity": "critical"
        }})
    result["violations"] = mapped_violations
    result["parts"] = part_reports
    
    result["cost_analysis"]["total_cost"] = total_cost
    result["cost_analysis"]["unit_cost"] = total_cost / {quantity} if {quantity} > 0 else 0.0

except Exception as e:
    result["error"] = str(e)

print(f"ANALYSIS_RESULT:{{json.dumps(result)}}")
"""

        try:
            runner_path.write_text(runner_script, encoding="utf-8")

            # Execute in sandbox
            stdout, stderr, returncode = self.sandbox.run_script(
                runner_filename, session_id=self.active_session_id, mount_src=True
            )

            if runner_path.exists():
                runner_path.unlink()

            if "ANALYSIS_RESULT:" in stdout:
                try:
                    output_line = [
                        line
                        for line in stdout.split("\n")
                        if "ANALYSIS_RESULT:" in line
                    ][0]
                    result_json = output_line.split("ANALYSIS_RESULT:")[1].strip()
                    return json.loads(result_json)
                except Exception as e:
                    return {
                        "error": f"Failed to parse analysis result: {e!s}",
                        "raw_output": stdout,
                    }
            else:
                return {
                    "error": f"Analysis execution failed (no result key found).",
                    "raw_output": stdout + stderr,
                }

        except Exception as e:
            return {"error": f"Host error during analysis: {e!s}"}

    def dispatch(self, tool_name: str, arguments: Dict[str, Any]) -> str:
        """Dispatches a tool call to the appropriate method."""
        tool_map = {
            "write_file": self.write_file,
            "edit_file": self.edit_file,
            "preview_design": self.preview_design,
            "search_docs": self.search_docs,
            "search_parts": self.search_parts,
            "preview_part": self.preview_part,
            "check_manufacturability": self.check_manufacturability,
            "run_command": self.run_command,
            "lint_script": self.lint_script,
            "list_skills": self.list_skills,
            "list_skill_files": self.list_skill_files,
            "read_skill": self.read_skill,
            "update_skill": self.update_skill,
            "init_skill": self.init_skill,
            "package_skill": self.package_skill,
            "run_skill_script": self.run_skill_script,
            "start_session": self.start_session,
            "stop_session": self.stop_session,
        }

        func = tool_map.get(tool_name)
        if not func:
            return f"Unknown tool: {tool_name}"

        # Handle simplified arguments if necessary, but assuming core.py passes correct kwargs
        # Some tools have specific arg names that might differ from generic 'value' or 'query'
        # But core.py usually maps them.

        try:
            # Special handling for tools where we want to ensure return type is string for the environment observation
            result = func(**arguments)
            if isinstance(result, (dict, list)):
                return json.dumps(result)
            return str(result)
        except TypeError as e:
            # Fallback for single argument tools called with 'value' or 'query'
            val = (
                arguments.get("value")
                or arguments.get("query")
                or arguments.get("filename")
                or arguments.get("command")
            )
            if (
                val is not None and len(arguments) <= 2
            ):  # Allow for potential extra args
                try:
                    result = func(val)
                    if isinstance(result, (dict, list)):
                        return json.dumps(result)
                    return str(result)
                except Exception as e2:
                    return f"Error executing tool {tool_name}: {e!s} (Fallback failed: {e2!s})"
            return f"Error executing tool {tool_name}: {e!s}"
        except Exception as e:
            return f"Error executing tool {tool_name}: {e!s}"
