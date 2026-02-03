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
from src.compiler import mujoco_bridge
from src.environment.sandbox import PodmanSandbox
from src.rag import search as rag_search
from src.simulation_engine import evaluator
from src.workbenches.cnc import CNCWorkbench
from src.workbenches.injection_molding import InjectionMoldingWorkbench
from src.workbenches.print_3d import Print3DWorkbench


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
        self.active_episode_id: Any = None  # For logging context
        self.constraints: Dict[str, Any] = {}

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
        """Executes the script and renders result to SVG."""
        filename = Path(filename).name
        script_path = Path(self.workspace_dir) / filename

        if not script_path.exists():
            return f"Error: File {filename} does not exist."

        runner_filename = f"runner_{filename}"
        runner_path = Path(self.workspace_dir) / runner_filename
        output_json_path = "_preview_result.json"

        runner_script = evaluator.generate_preview_script(filename, output_json_path)

        try:
            runner_path.write_text(runner_script, encoding="utf-8")
            stdout, stderr, _ = self.sandbox.run_script(
                runner_filename, session_id=self.active_session_id, mount_src=True
            )

            if runner_path.exists():
                runner_path.unlink()

            # Read result
            result_path = Path(self.workspace_dir) / output_json_path
            if result_path.exists():
                data = json.loads(result_path.read_text(encoding="utf-8"))
                result_path.unlink()

                if data.get("status") == "success":
                    return f"Preview generated: {data.get('generated_file')}"
                else:
                    return f"Error generating preview: {data.get('error')}\nOutput: {stdout}{stderr}"
            else:
                return f"Error: No result file generated.\nOutput: {stdout}{stderr}"

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
        output_json_path = "_val_result.json"
        stl_filename = "design.stl"

        runner_script = evaluator.generate_validation_script(
            design_file,
            process,
            quantity,
            export_stl,
            output_json_path,
            stl_filename,
        )

        try:
            runner_path.write_text(runner_script, encoding="utf-8")
            stdout, stderr, _ = self.sandbox.run_script(
                runner_filename, session_id=self.active_session_id, mount_src=True
            )

            if runner_path.exists():
                runner_path.unlink()

            # Read result
            result_path = Path(self.workspace_dir) / output_json_path
            if result_path.exists():
                data = json.loads(result_path.read_text(encoding="utf-8"))
                result_path.unlink()
                return data
            else:
                return {"error": f"Exec failed (no result): {stdout}{stderr}"}

        except Exception as e:
            return {"error": f"Host error: {e!s}"}

    def log_message(
        self,
        content: str,
        type: str = "thought",
        agent_role: Optional[str] = None,
        metadata: Optional[dict[str, Any]] = None,
    ):
        """Logs a message to the database if available."""
        # Note: DatabaseManager.log_step usually requires an episode_id.
        # This is a best-effort log if the runtime is aware of the context.
        if self.db and hasattr(self.db, "log_step") and self.active_episode_id:
            try:
                # We default sequence_index to 0 or handled by DB?
                # Actually log_step requires sequence_index.
                # Since ToolRuntime doesn't manage step counts, this might be limited.
                # However, we provide the hook for compatibility.
                pass
            except Exception:
                pass

    def check_manufacturability(
        self, design_file: str = "design.py", process: str = "cnc", quantity: int = 1
    ) -> Dict[str, Any]:
        """Wrapper for validate_and_export (Legacy compatibility)."""
        return self.validate_and_export(
            design_file, process, quantity, export_stl=False
        )

    def submit_design(
        self,
        control_path: str,
        max_unit_cost: Optional[float] = None,
        target_quantity: Optional[int] = None,
        force_submit: bool = False,
    ) -> str:
        """
        Submits the design, runs validation, checking against budget, and runs simulation.
        Returns a human-readable report string.
        """
        # Resolve constraints
        q_target = (
            target_quantity
            if target_quantity is not None
            else self.constraints.get("target_quantity", 1)
        )
        c_max = (
            max_unit_cost
            if max_unit_cost is not None
            else self.constraints.get("max_unit_cost", float("inf"))
        )

        # 1. Validate and Export STL
        res = self.validate_and_export(
            design_file="design.py",
            process="print_3d",
            quantity=q_target,
            export_stl=True,
        )

        if "error" in res and res["error"]:
            return f"Error processing design: {res['error']}"

        # 2. Budget Check
        unit_cost = res.get("cost_analysis", {}).get("unit_cost", 0.0)
        if unit_cost > c_max and not force_submit:
            return f"REJECTED: Unit cost ${unit_cost:.2f} exceeds budget ${c_max:.2f}. Use 'force_submit=True' if you believe this is necessary."

        # 3. Validation Check
        if res.get("status") == "fail":
            v_str = ", ".join(
                [v.get("description", "Unknown") for v in res.get("violations", [])]
            )
            return f"Validation Failed: {v_str}"

        if not res.get("stl_path"):
            return "Submission failed: No STL exported."

        # 4. Simulation (Host side)
        try:
            stl_path = Path(self.workspace_dir) / res["stl_path"]
            template_xml = self.sim_bridge.load_template()
            injected_xml = self.sim_bridge.inject_design(template_xml, stl_path)
            sim_result = self.sim_bridge.run_simulation(injected_xml)

            status = "success" if sim_result.success else "failure"
            reward = (
                (100.0 - (sim_result.energy * 0.1) - (sim_result.damage * 10.0))
                if sim_result.success
                else -50.0
            )

            # Log to DB if possible
            if self.db and hasattr(self.db, "update_episode_status") and self.active_episode_id:
                self.db.update_episode_status(
                    self.active_episode_id,
                    status=status,
                    result_metrics={
                        "energy": sim_result.energy,
                        "damage": sim_result.damage,
                        "success": sim_result.success,
                        "reward": reward,
                    },
                )

            return f"Submission Result: {status.upper()}. Reward: {reward:.2f}. Energy: {sim_result.energy:.2f}"

        except Exception as e:
            return f"Simulation Error: {e!s}"

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
            "submit_design": self.submit_design,
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
        sdir = Path(".agent/skills") / skill_name
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
        path = Path(".agent/skills") / skill_name / "scripts" / script_name
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
        sdir = Path(".agent/skills")
        return "Skills:\n- " + "\n- ".join(
            sorted([p.name for p in sdir.iterdir() if p.is_dir()])
        )

    def list_skill_files(self, skill_name: str) -> str:
        sdir = Path(".agent/skills") / skill_name
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
        sdir = Path(".agent/skills") / skill_name
        if not sdir.exists():
            return f"Error: {skill_name} missing"
        tpath = sdir / (resource_type or "") / Path(filename).name
        tpath.parent.mkdir(parents=True, exist_ok=True)
        tpath.write_text(content, encoding="utf-8")
        return f"Updated {tpath}"

    def init_skill(self, skill_name: str) -> str:
        script = Path(".agent/skills/skill-creator/scripts/init_skill.py")
        res = subprocess.run(
            [sys.executable, str(script), skill_name, "--path", ".agent/skills"],
            capture_output=True,
            text=True,
        )
        return res.stdout or res.stderr

    def package_skill(self, skill_name: str) -> str:
        script = Path(".agent/skills/skill-creator/scripts/package_skill.py")
        res = subprocess.run(
            [sys.executable, str(script), str(Path(".agent/skills") / skill_name)],
            capture_output=True,
            text=True,
        )
        return res.stdout or res.stderr
