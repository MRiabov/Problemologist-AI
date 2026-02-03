import json
from pathlib import Path
from typing import Any

from src.agent.utils.config import Config
from src.agent.utils.linter import format_linter_report, run_linter
from src.compiler import mujoco_bridge
from src.compiler.models import ValidationReport
from src.cots.core import PartIndex
from src.cots.providers.bd_warehouse import BDWarehouseProvider
from src.environment.evaluator import Evaluator
from src.environment.persistence import DatabaseManager
from src.environment.sandbox import PodmanSandbox
from src.rag import search as rag_search
from src.workbenches.cnc import CNCWorkbench
from src.workbenches.injection_molding import InjectionMoldingWorkbench
from src.workbenches.print_3d import Print3DWorkbench


class ToolRuntime:
    """
    Encapsulates runtime state and tool implementations for the CAD environment.
    Replaces global state from the legacy tools.py module.
    """

    def __init__(self, workspace_dir: str, db: DatabaseManager | None = None):
        self.workspace_dir = str(Path(workspace_dir).resolve())
        Path(self.workspace_dir).mkdir(parents=True, exist_ok=True)

        self.sandbox = PodmanSandbox(self.workspace_dir)
        self.evaluator = Evaluator(self.sandbox, self.workspace_dir)
        self.db = db

        self.active_session_id: str | None = None

        # Simulation
        self.sim_bridge = mujoco_bridge.MujocoBridge()

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
        if self.active_session_id and self.sandbox.stop_session(self.active_session_id):
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

    def preview_design(self, filename: str = "design.py") -> str:
        """Executes the script and renders result to SVG."""
        return self.evaluator.preview_design(
            filename, session_id=self.active_session_id
        )

    def validate_and_export(
        self,
        design_file: str = "design.py",
        process: str = "cnc",
        quantity: int = 1,
        export_stl: bool = False,
    ) -> ValidationReport:
        """Validates design for manufacturability and optionally exports STL."""
        return self.evaluator.validate_and_export(
            design_file=design_file,
            process=process,
            quantity=quantity,
            export_stl=export_stl,
            session_id=self.active_session_id,
        )

    def check_manufacturability(
        self, design_file: str = "design.py", process: str = "cnc", quantity: int = 1
    ) -> ValidationReport:
        """Wrapper for validate_and_export (Legacy compatibility)."""
        return self.validate_and_export(
            design_file, process, quantity, export_stl=False
        )

    def submit_design(
        self,
        control_path: str,
        design_file: str = "design.py",
        process: str = "print_3d",
        target_quantity: int = 1,
        max_unit_cost: float = float("inf"),
        force_submit: bool = False,
    ) -> dict[str, Any]:
        """
        Runs the current design script, performs full Workbench validation,
        and optionally runs a host-side simulation.
        """
        temp_file = Path(design_file).name
        try:
            (Path(self.workspace_dir) / temp_file).read_text(encoding="utf-8")
        except Exception as e:
            return {"status": "error", "message": f"Could not read script: {e}"}

        # 1. Validate and Export STL
        res: ValidationReport = self.validate_and_export(
            design_file=design_file,
            process=process,
            quantity=target_quantity,
            export_stl=True,
        )

        if res.error:
            return {
                "status": "fail",
                "reward": -10.0,
                "message": f"Error processing design: {res.error}",
                "terminated": False,
            }

        # 2. Budget Check
        unit_cost = res.cost_analysis.unit_cost
        if unit_cost > max_unit_cost and not force_submit:
            return {
                "status": "fail",
                "reward": -20.0,
                "message": f"REJECTED: Unit cost ${unit_cost:.2f} exceeds budget ${max_unit_cost:.2f}.",
                "terminated": False,
            }

        # 3. Validation Check
        if res.status == "fail":
            v_str = ", ".join([v.description for v in res.violations])
            return {
                "status": "fail",
                "reward": -10.0,
                "message": f"Validation Failed: {v_str}",
                "terminated": False,
            }

        if not res.stl_path:
            return {
                "status": "fail",
                "reward": -10.0,
                "message": "Submission failed: No STL exported.",
                "terminated": False,
            }

        # 4. Save to Database for Observability if persistence is active
        # Episode/Step logic should ideally be handled at the agent level or injected
        # For now, we update best cost if possible
        if self.db:
            # If db is passed, it should be a DatabaseManager
            self.db.update_cost_record(
                scenario_id=control_path,
                unit_cost=unit_cost,
                episode_id=self.active_session_id,
            )

        # 5. Simulation (Host side)
        try:
            stl_path = Path(self.workspace_dir) / res.stl_path
            template_xml = self.sim_bridge.load_template()
            injected_xml = self.sim_bridge.inject_design(template_xml, stl_path)
            sim_result = self.sim_bridge.run_simulation(injected_xml)

            reward = (
                (100.0 - (sim_result.energy * 0.1) - (sim_result.damage * 10.0))
                if sim_result.success
                else -50.0
            )
            status = "success" if sim_result.success else "failure"

            return {
                "status": status,
                "reward": reward,
                "message": f"Submission Result: {status.upper()}. Energy: {sim_result.energy:.2f}",
                "terminated": True,
                "metrics": {
                    "energy": sim_result.energy,
                    "damage": sim_result.damage,
                    "success": sim_result.success,
                },
            }

        except Exception as e:
            return {
                "status": "error",
                "reward": -10.0,
                "message": f"Simulation Error: {e!s}",
                "terminated": False,
            }

    def dispatch(self, tool_name: str, arguments: dict[str, Any]) -> str:
        """Dispatches a tool call to the appropriate method."""
        tool_map = {
            "write_file": self.write_file,
            "edit_file": self.edit_file,
            "preview_design": self.preview_design,
            "submit_design": self.submit_design,
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
        resource_type: str | None = None,
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
        """Executes a specialized script located within a skill inside the sandbox."""
        import shlex

        # Skill scripts are mounted at docs/skills/ inside the sandbox
        script_path = f"docs/skills/{skill_name}/scripts/{script_name}"
        args = shlex.split(arguments)
        cmd = ["python3", script_path] + args

        if self.active_session_id:
            stdout, stderr, rc = self.sandbox.exec_command(self.active_session_id, cmd)
        else:
            # Fallback to transient run with skills mounted
            stdout, stderr, rc = self.sandbox.run_script(
                "-c",
                f"import subprocess, sys; r=subprocess.run({cmd}, capture_output=True, text=True); print(r.stdout); print(r.stderr, file=sys.stderr); sys.exit(r.returncode)",
                mount_src=True,
                extra_mounts=[(str(Config.SKILLS_DIR), "/workspace/docs/skills")],
            )
        return f"{stdout}{stderr}\nReturn Code: {rc}"

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
        resource_type: str | None = None,
    ) -> str:
        """Updates a skill file on host after (optional) sandboxed validation."""
        # TODO: Implement sandboxed linting/validation here before writing to host
        sdir = Config.SKILLS_DIR / skill_name
        if not sdir.exists():
            return f"Error: {skill_name} missing"
        tpath = sdir / (resource_type or "") / Path(filename).name
        tpath.parent.mkdir(parents=True, exist_ok=True)
        tpath.write_text(content, encoding="utf-8")
        return f"Updated {tpath} (Host-side)"

    def init_skill(self, skill_name: str) -> str:
        """Initializes a new skill using the skill-creator inside the sandbox."""
        script_path = "docs/skills/skill-creator/scripts/init_skill.py"
        cmd = ["python3", script_path, skill_name, "--path", "/workspace/docs/skills"]

        if self.active_session_id:
            stdout, stderr, rc = self.sandbox.exec_command(self.active_session_id, cmd)
        else:
            stdout, stderr, rc = self.sandbox.run_script(
                "-c",
                f"import subprocess, sys; r=subprocess.run({cmd}, capture_output=True, text=True); print(r.stdout); print(r.stderr, file=sys.stderr); sys.exit(r.returncode)",
                mount_src=True,
                extra_mounts=[(str(Config.SKILLS_DIR), "/workspace/docs/skills")],
            )
        return f"{stdout}{stderr}"

    def package_skill(self, skill_name: str) -> str:
        """Packages a skill using the skill-creator inside the sandbox."""
        script_path = "docs/skills/skill-creator/scripts/package_skill.py"
        target_path = f"/workspace/docs/skills/{skill_name}"
        cmd = ["python3", script_path, target_path]

        if self.active_session_id:
            stdout, stderr, rc = self.sandbox.exec_command(self.active_session_id, cmd)
        else:
            stdout, stderr, rc = self.sandbox.run_script(
                "-c",
                f"import subprocess, sys; r=subprocess.run({cmd}, capture_output=True, text=True); print(r.stdout); print(r.stderr, file=sys.stderr); sys.exit(r.returncode)",
                mount_src=True,
                extra_mounts=[(str(Config.SKILLS_DIR), "/workspace/docs/skills")],
            )
        return f"{stdout}{stderr}"
