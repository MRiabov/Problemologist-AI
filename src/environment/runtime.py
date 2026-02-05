import json
import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

from src.agent.utils.config import Config
from src.agent.utils.linter import format_linter_report, run_linter
from src.simulation_engine import bridge as mujoco_bridge
from src.workbenches.models import ValidationReport
from src.cots.core import PartIndex
from src.cots.providers.bd_warehouse import BDWarehouseProvider
from src.environment.design_executor import DesignExecutor
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

    def __init__(
        self,
        workspace_dir: str,
        db: DatabaseManager | None = None,
        problem_id: str = "default_problem",
    ):
        self.workspace_dir = str(Path(workspace_dir).resolve())
        Path(self.workspace_dir).mkdir(parents=True, exist_ok=True)

        self.sandbox = PodmanSandbox(self.workspace_dir)
        self.design_executor = DesignExecutor(self.sandbox, self.workspace_dir)
        self.db = db
        self.problem_id = problem_id

        self.episode = None
        self.step_count = 0
        self.active_session_id: str | None = None

        # Simulation
        self.sim_bridge = mujoco_bridge.MujocoBridge(
            workspace_dir=self.workspace_dir,
            sandbox=self.sandbox,
        )

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
        """Starts a persistent sandbox session and initializes an episode."""
        if self.sandbox.start_session(session_id):
            self.active_session_id = session_id

            # Episode management (Persistence)
            if self.db:
                self.episode = self.db.create_episode(self.problem_id)
                self.step_count = 0

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
        if path.startswith("docs/skills/"):
            skill_rel = path.removeprefix("docs/skills/")
            full_path = Config.SKILLS_DIR / skill_rel
            # Ensure we are writing inside SKILLS_DIR
            if not str(full_path.resolve()).startswith(
                str(Config.SKILLS_DIR.resolve())
            ):
                return (
                    f"Error: Path {path} attempts to write outside skills directory."
                )
        else:
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
        if path.startswith("docs/skills/"):
            skill_rel = path.removeprefix("docs/skills/")
            full_path = Config.SKILLS_DIR / skill_rel
            # Ensure we are writing inside SKILLS_DIR
            if not str(full_path.resolve()).startswith(
                str(Config.SKILLS_DIR.resolve())
            ):
                return (
                    f"Error: Path {path} attempts to write outside skills directory."
                )
        else:
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
        return self.design_executor.preview_design(
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
        return self.design_executor.validate_and_export(
            design_file=design_file,
            process=process,
            quantity=quantity,
            export_stl=export_stl,
            session_id=self.active_session_id,
        )

    def analyze_design(
        self, design_file: str = "design.py", process: str = "cnc", quantity: int = 1
    ) -> ValidationReport:
        """
        Runs detailed manufacturability and cost analysis without submitting.
        Renamed from check_manufacturability.
        """
        return self.validate_and_export(
            design_file, process, quantity, export_stl=False
        )

    def verify_solution(
        self,
        control_path: str,
        design_file: str = "design.py",
        process: str = "print_3d",
        target_quantity: int = 1,
        max_unit_cost: float = float("inf"),
        force_submit: bool = False,
    ) -> dict[str, Any]:
        """
        Runs the full verification pipeline: DFM checks, budget validation, and simulation.
        Renamed from submit_design.
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
                (
                    100.0
                    - (sim_result.total_energy * 0.1)
                    - (sim_result.total_damage * 10.0)
                )
                if sim_result.success
                else -50.0
            )
            status = "success" if sim_result.success else "failure"

            return {
                "status": status,
                "reward": reward,
                "message": f"Submission Result: {status.upper()}. Energy: {sim_result.total_energy:.2f}",
                "terminated": True,
                "metrics": {
                    "energy": sim_result.total_energy,
                    "damage": sim_result.total_damage,
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

    def _run_tool(self, tool_name: str, arguments: dict[str, Any]) -> Any:
        """Internal helper to find and execute a tool function."""
        tool_map = {
            "write_file": self.write_file,
            "edit_file": self.edit_file,
            "preview_design": self.preview_design,
            "verify_solution": self.verify_solution,
            "search_docs": self.search_docs,
            "search_parts": self.search_parts,
            "preview_part": self.preview_part,
            "analyze_design": self.analyze_design,
            "run_command": self.run_command,
            "lint_script": self.lint_script,
            "start_session": self.start_session,
            "stop_session": self.stop_session,
        }

        func = tool_map.get(tool_name)
        if not func:
            raise ValueError(f"Unknown tool: {tool_name}")

        return func(**arguments)

    def dispatch(
        self, tool_name: str, arguments: dict[str, Any], agent_role: str | None = None
    ) -> str:
        """Dispatches a tool call, handles persistence, and returns a serialized string."""
        import time

        start_time = time.time()
        tool_output = ""

        try:
            result = self._run_tool(tool_name, arguments)

            if hasattr(result, "model_dump_json"):
                tool_output = result.model_dump_json()
            elif hasattr(result, "model_dump"):
                tool_output = json.dumps(result.model_dump())
            elif hasattr(result, "__dataclass_fields__"):
                from dataclasses import asdict

                tool_output = json.dumps(asdict(result))
            elif isinstance(result, (dict, list)):
                tool_output = json.dumps(result)
            else:
                tool_output = str(result)
        except Exception as e:
            import traceback

            logger.error(
                f"Error executing tool {tool_name}: {e!s}\n{traceback.format_exc()}"
            )
            tool_output = f"Error executing tool {tool_name}: {e!s}"

        duration_ms = int((time.time() - start_time) * 1000)

        # 2. Persistence (Consolidated from CADEnv)
        if self.db and self.episode:
            self.step_count += 1
            db_step = self.db.log_step(
                episode_id=self.episode.id,
                sequence_index=self.step_count,
                tool_name=tool_name,
                tool_input=json.dumps(arguments),
                tool_output=tool_output,
                duration_ms=duration_ms,
                agent_role=agent_role,
            )

            # Special case for artifacts
            if tool_name == "preview_design" and "Preview generated:" in tool_output:
                try:
                    artifact_path = tool_output.split("Preview generated:")[1].strip()
                    self.db.save_artifact(
                        step_id=db_step.id,
                        artifact_type="image",
                        file_path=artifact_path,
                    )
                except IndexError:
                    pass

        return tool_output

    def log_message(
        self,
        content: str,
        msg_type: str = "thought",
        agent_role: str | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> None:
        """Logs a message to the database (persistence layer)."""
        if self.db and self.episode:
            self.step_count += 1
            self.db.log_step(
                episode_id=self.episode.id,
                sequence_index=self.step_count,
                tool_name="log_message",
                tool_input=json.dumps({"type": msg_type, "metadata": metadata or {}}),
                tool_output=content,
                duration_ms=0,
                agent_role=agent_role,
            )

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
