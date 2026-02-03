import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import build123d as bd

from src.agent.utils.linter import format_linter_report, run_linter
from src.cots.core import PartIndex
from src.cots.providers.bd_warehouse import BDWarehouseProvider
from src.environment.sandbox import PodmanSandbox
from src.rag import search as rag_search
from src.workbenches.cnc import CNCWorkbench
from src.workbenches.injection_molding import InjectionMoldingWorkbench
from src.workbenches.print_3d import Print3DWorkbench
from src.agent.utils.config import Config


class ToolRuntime:
    """
    Encapsulates runtime state and tool implementations for the CAD environment.
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

                    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    content = f"\n\n## [{ts}]\n{content}\n"

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
        """Replaces a unique 'find' string with 'replace' in the specified file."""
        full_path = Path(self.workspace_dir) / path

        if not full_path.exists():
            return f"Error: File {path} does not exist."

        try:
            content = full_path.read_text(encoding="utf-8")

            count = content.count(find)
            if count == 0:
                return f"Error: 'find' string not found in {path}."
            if count > 1:
                return f"Error: 'find' string is ambiguous ({count} found) in {path}."

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
        """Reads content of any file in workspace or mounted docs."""
        if path.startswith("docs/skills/"):
            skill_rel = path.replace("docs/skills/", "")
            host_path = Config.SKILLS_DIR / skill_rel
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

    def _get_runner_script_prefix(self, filename: str) -> str:
        return f"""
import sys
import json
import hashlib
from pathlib import Path
import build123d as bd

sys.path.append("/workspace")

def get_export_obj(locs):
    for val in locs.values():
        if isinstance(val, (bd.Compound, bd.Solid, bd.Shape)):
            return val
        elif hasattr(val, "part") and isinstance(val.part, bd.Shape):
            return val.part
        elif hasattr(val, "sketch") and isinstance(val.sketch, bd.Shape):
            return val.sketch
        elif hasattr(val, "line") and isinstance(val.line, bd.Shape):
            return val.line
    return None

def parse_label(label, default_q, default_p):
    data = {{"quantity": default_q, "process": default_p}}
    if not label: return data
    for p in str(label).split("|"):
        if ":" in p:
            k, v = p.split(":", 1)
            if k == "quantity": 
                try: data["quantity"] = int(v)
                except (ValueError, TypeError): pass
            elif k == "process": data["process"] = v
    return data
"""

    def preview_design(self, filename: str = "design.py") -> str:
        """Executes the script and renders result to SVG."""
        filename = Path(filename).name
        script_path = Path(self.workspace_dir) / filename

        if not script_path.exists():
            return f"Error: File {filename} does not exist."

        runner_filename = f"runner_{filename}"
        runner_path = Path(self.workspace_dir) / runner_filename

        runner_script = (
            self._get_runner_script_prefix(filename)
            + f"""
from build123d import ExportSVG, Drawing, Unit

try:
    locs = {{}}
    with Path("/workspace/{filename}").open("r", encoding="utf-8") as f:
        code = f.read()
    exec(code, globals(), locs)
    
    export_obj = get_export_obj(locs)
            
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
        print("RENDER_ERROR: No 3D object found")
except Exception as e:
    print(f"RENDER_EXCEPTION:{{str(e)}}")
"""
        )

        try:
            runner_path.write_text(runner_script, encoding="utf-8")
            stdout, stderr, _ = self.sandbox.run_script(
                runner_filename, session_id=self.active_session_id, mount_src=True
            )

            if runner_path.exists():
                runner_path.unlink()

            if "RENDER_SUCCESS" in stdout:
                line = [l for l in stdout.split("\n") if "RENDER_SUCCESS" in l][0]
                gen_file = line.split("RENDER_SUCCESS:")[1].strip()
                return f"Preview generated: {gen_file}"

            return f"Error generating preview: {stdout}{stderr}"

        except Exception as e:
            return f"Error: {e!s}"

    def validate_and_export(
        self,
        design_file: str = "design.py",
        process: str = "cnc",
        quantity: int = 1,
        export_stl: bool = False,
    ) -> Dict[str, Any]:
        """Validates design for manufacturability and optionally exports STL."""
        design_file = Path(design_file).name
        script_path = Path(self.workspace_dir) / design_file

        if not script_path.exists():
            return {"error": f"File {design_file} does not exist."}

        runner_filename = (
            f"val_runner_{hashlib.md5(design_file.encode()).hexdigest()[:8]}.py"
        )
        runner_path = Path(self.workspace_dir) / runner_filename
        stl_filename = "design.stl"

        runner_script = (
            self._get_runner_script_prefix(design_file)
            + f"""
from src.workbenches.cnc import CNCWorkbench
from src.workbenches.injection_molding import InjectionMoldingWorkbench
from src.workbenches.print_3d import Print3DWorkbench
from src.compiler import geometry

result = {{
    "status": "fail", 
    "manufacturability_score": 0.0,
    "violations": [],
    "parts": [],
    "cost_analysis": {{"total_cost": 0.0, "unit_cost": 0.0, "target_quantity": {quantity}}},
    "stl_path": None,
    "error": None
}}

try:
    locs = {{}}
    with Path("/workspace/{design_file}").open("r", encoding="utf-8") as f:
        code = f.read()
    exec(code, globals(), locs)
    
    export_obj = get_export_obj(locs)
    if not export_obj:
        raise ValueError("No 3D object found")

    all_solids = list(export_obj.solids()) if isinstance(export_obj, bd.Compound) else [export_obj]

    workbenches = {{
        "cnc": CNCWorkbench(), "injection_molding": InjectionMoldingWorkbench(), "print_3d": Print3DWorkbench()
    }}
    
    total_cost, all_violations, part_reports, reuse_ctx = 0.0, [], [], {{}}

    for i, solid in enumerate(all_solids):
        ldata = parse_label(getattr(solid, "label", ""), {quantity}, "{process}")
        wb = workbenches.get(ldata["process"], workbenches.get("{process}", workbenches["print_3d"]))
        
        violations = wb.validate(solid)
        all_violations.extend(violations)
        cost = wb.calculate_cost(solid, ldata["quantity"], context=reuse_ctx)
        total_cost += cost
        
        part_reports.append({{
            "part_index": i, "process": ldata["process"], "quantity": ldata["quantity"],
            "cost": cost, "violations": [str(v) for v in violations]
        }})
        
    result["status"] = "pass" if not all_violations else "fail"
    result["manufacturability_score"] = max(0.0, 1.0 - len(all_violations)*0.1)
    result["violations"] = [{{"description": str(v)}} for v in all_violations]
    result["parts"], result["cost_analysis"]["total_cost"] = part_reports, total_cost
    result["cost_analysis"]["unit_cost"] = total_cost / {quantity} if {quantity} > 0 else 0.0

    if {export_stl} and not all_violations:
        geometry.export_mesh(export_obj, "/workspace/{stl_filename}")
        result["stl_path"] = "{stl_filename}"

except Exception as e:
    result["error"] = str(e)

print(f"VAL_RESULT:{{json.dumps(result)}}")
"""
        )

        try:
            runner_path.write_text(runner_script, encoding="utf-8")
            stdout, stderr, _ = self.sandbox.run_script(
                runner_filename, session_id=self.active_session_id, mount_src=True
            )

            if runner_path.exists():
                runner_path.unlink()

            if "VAL_RESULT:" in stdout:
                line = [l for l in stdout.split("\n") if "VAL_RESULT:" in l][0]
                return json.loads(line.split("VAL_RESULT:")[1].strip())

            return {"error": f"Exec failed: {stdout}{stderr}"}

        except Exception as e:
            return {"error": f"Host error: {e!s}"}

    def check_manufacturability(
        self, design_file: str = "design.py", process: str = "cnc", quantity: int = 1
    ) -> Dict[str, Any]:
        """Wrapper for validate_and_export (Legacy compatibility)."""
        return self.validate_and_export(
            design_file, process, quantity, export_stl=False
        )

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

        try:
            result = func(**arguments)
            return (
                json.dumps(result) if isinstance(result, (dict, list)) else str(result)
            )
        except Exception as e:
            return f"Error executing tool {tool_name}: {e!s}"

    # Handing remaining methods (search_docs, etc.) by delegating to originals or simplified versions
    def search_docs(self, query: str) -> str:
        return rag_search.search(query)

    def search_parts(self, query: str) -> str:
        try:
            results = self.part_index.search(query)
            if not results:
                return f"No parts found for: {query}"
            return "Found parts:\n- " + "\n- ".join(
                [f"{r.id}: {r.name}" for r in results]
            )
        except Exception as e:
            return f"Error: {e!s}"

    def preview_part(self, part_id: str) -> str:
        try:
            p = self.part_index.preview(part_id)
            return f"Part: {p.id}\nRecipe:\n```python\n{p.recipe}\n```"
        except Exception as e:
            return f"Error: {e!s}"

    def run_command(self, command: str, timeout: int = 60) -> str:
        import shlex

        args = shlex.split(command)
        if not self.active_session_id:
            return "Error: No active session."
        stdout, stderr, rc = self.sandbox.exec_command(
            self.active_session_id, args, timeout=timeout
        )
        return f"{stdout}{stderr}\nReturn Code: {rc}"

    def lint_script(self, filename: str = "design.py") -> str:
        path = Path(self.workspace_dir) / Path(filename).name
        if not path.exists():
            return f"Error: {filename} missing"
        try:
            errors = run_linter(path.read_text(encoding="utf-8"))
            return "Clean!" if not errors else format_linter_report(errors)
        except Exception as e:
            return f"Error: {e!s}"

    def read_skill(
        self,
        skill_name: str,
        filename: str = "SKILL.md",
        resource_type: Optional[str] = None,
    ) -> str:
        sdir = Config.SKILLS_DIR / skill_name
        if not sdir.exists():
            return f"Error: {skill_name} missing"
        tpath = (
            sdir / resource_type / Path(filename).name
            if resource_type
            else sdir / Path(filename).name
        )
        if not tpath.exists():
            return f"Error: {filename} missing"
        return tpath.read_text(encoding="utf-8")

    def run_skill_script(
        self, skill_name: str, script_name: str, arguments: str = ""
    ) -> str:
        path = Config.SKILLS_DIR / skill_name / "scripts" / script_name
        if not path.exists():
            return f"Error: {script_name} missing"
        try:
            cmd = [sys.executable, str(path)]
            if arguments:
                import shlex

                cmd.extend(shlex.split(arguments))
            res = subprocess.run(cmd, capture_output=True, text=True, check=False)
            return f"{res.stdout}{res.stderr}"
        except Exception as e:
            return f"Error: {e!s}"

    def list_skills(self) -> str:
        sdir = Config.SKILLS_DIR
        return "Skills:\n- " + "\n- ".join(
            sorted([p.name for p in sdir.iterdir() if p.is_dir()])
        )

    def list_skill_files(self, skill_name: str) -> str:
        sdir = Config.SKILLS_DIR / skill_name
        if not sdir.exists():
            return f"Error: {skill_name} missing"
        return "\n".join(
            [str(p.relative_to(sdir)) for p in sdir.rglob("*") if p.is_file()]
        )

    def update_skill(
        self,
        skill_name: str,
        content: str,
        filename: str = "SKILL.md",
        resource_type: Optional[str] = None,
    ) -> str:
        sdir = Config.SKILLS_DIR / skill_name
        if not sdir.exists():
            return f"Error: {skill_name} missing"
        tpath = sdir / (resource_type or "") / Path(filename).name
        tpath.parent.mkdir(parents=True, exist_ok=True)
        tpath.write_text(content, encoding="utf-8")
        return f"Updated {tpath}"

    def init_skill(self, skill_name: str) -> str:
        script = Config.SKILL_CREATOR_DIR / "scripts/init_skill.py"
        res = subprocess.run(
            [sys.executable, str(script), skill_name, "--path", str(Config.SKILLS_DIR)],
            capture_output=True,
            text=True,
        )
        return res.stdout or res.stderr

    def package_skill(self, skill_name: str) -> str:
        script = Config.SKILL_CREATOR_DIR / "scripts/package_skill.py"
        res = subprocess.run(
            [sys.executable, str(script), str(Config.SKILLS_DIR / skill_name)],
            capture_output=True,
            text=True,
        )
        return res.stdout or res.stderr
