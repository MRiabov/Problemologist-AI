import json
import time
from pathlib import Path
from typing import Any

from src.compiler import mujoco_bridge
from src.environment import persistence
from src.environment.runtime import ToolRuntime


class CADEnv:
    """
    Agentic CAD Environment.
    Implements an interface for LLM-based mechanical engineering agents.
    """

    def __init__(
        self,
        problem_id: str = "default_problem",
        task_description: str = "Design a part that satisfies geometric constraints.",
        db_url: str = "sqlite:///history.db",
        workspace_dir: str = "workspace",
        max_unit_cost: float = float("inf"),
        target_quantity: int = 1,
    ):
        self.problem_id = problem_id
        self.task_description = task_description
        self.max_unit_cost = max_unit_cost
        self.target_quantity = target_quantity

        # Initialize Runtime
        self.runtime = ToolRuntime(workspace_dir, None)
        self.workspace_dir = self.runtime.workspace_dir  # Expose absolute path

        # Persistence
        self.db = persistence.DatabaseManager(db_url)
        self.db.create_tables()
        self.runtime.db = self.db  # Inject DB

        self.episode = None
        self.step_count = 0

        # Simulation
        self.sim_bridge = mujoco_bridge.MujocoBridge()

        self.last_obs = {
            "code": "",
            "last_output": "",
            "last_render": "",
            "task_description": self.task_description,
            "error": "",
            "budget_info": f"Target: {self.target_quantity}, Max Cost: ${self.max_unit_cost}",
        }

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        self.episode = self.db.create_episode(self.problem_id)
        self.step_count = 0

        # Reset design script
        script_path = Path(self.workspace_dir) / "design.py"
        script_path.write_text("", encoding="utf-8")

        self.last_obs.update(
            {
                "code": "",
                "last_output": "Environment reset. Ready for new design.",
                "last_render": "",
                "error": "",
            }
        )

        return self.last_obs, {}

    def dispatch(
        self, tool_name: str, arguments: dict[str, Any], agent_role: str | None = None
    ) -> str:
        """
        Dispatches a tool call, logs it to persistence, and returns the tool output.
        """
        self.step_count += 1
        start_time = time.time()

        tool_output = ""

        try:
            if tool_name == "submit_design":
                # Special handling for submission (reward, termination, simulation)
                reward, tool_output, terminated = self._submit_design(arguments)
                # We could log reward/terminated to tool_output or just persistence
                if terminated:
                    tool_output += f"\n\n[TERMINATED] Reward: {reward}"
            else:
                tool_output = self.runtime.dispatch(tool_name, arguments)

                # Update last_render if preview generated (for legacy/persistence)
                if (
                    tool_name == "preview_design"
                    and "Preview generated:" in tool_output
                ):
                    try:
                        self.last_obs["last_render"] = tool_output.split(
                            "Preview generated:"
                        )[1].strip()
                    except IndexError:
                        pass

        except Exception as e:
            tool_output = f"Error: {e!s}"
            self.last_obs["error"] = str(e)

        duration_ms = int((time.time() - start_time) * 1000)

        # Persistence
        db_step = self.db.log_step(
            episode_id=self.episode.id,
            sequence_index=self.step_count,
            tool_name=tool_name,
            tool_input=json.dumps(arguments),
            tool_output=tool_output,
            duration_ms=duration_ms,
            agent_role=agent_role,
        )

        if tool_name == "preview_design" and self.last_obs["last_render"]:
            self.db.save_artifact(
                step_id=db_step.id,
                artifact_type="image",
                file_path=self.last_obs["last_render"],
            )

        # Update last_obs (legacy but kept for internal state tracking)
        spath = Path(self.workspace_dir) / "design.py"
        if spath.exists():
            self.last_obs["code"] = spath.read_text(encoding="utf-8")
        self.last_obs["last_output"] = tool_output

        return tool_output

    def _submit_design(self, arguments: dict[str, Any]) -> tuple[float, str, bool]:
        """Handles submission via ToolRuntime validation and host-side simulation."""

        # 1. Validate and Export STL via Runtime (Securely)
        res = self.runtime.validate_and_export(
            design_file="design.py",
            process="print_3d",  # Default or from args?
            quantity=self.target_quantity,
            export_stl=True,
        )

        if "error" in res and res["error"]:
            return -10.0, f"Error processing design: {res['error']}", False

        # 2. Budget Check
        force_submit = arguments.get("force_submit", False)
        if isinstance(force_submit, str):
            force_submit = force_submit.lower() == "true"

        unit_cost = res.get("cost_analysis", {}).get("unit_cost", 0.0)
        if unit_cost > self.max_unit_cost and not force_submit:
            return (
                -20.0,
                f"REJECTED: Unit cost ${unit_cost:.2f} exceeds budget ${self.max_unit_cost:.2f}.",
                False,
            )

        # 3. Validation Check
        if res.get("status") == "fail":
            v_str = ", ".join(
                [v.get("description", "Unknown") for v in res.get("violations", [])]
            )
            return -10.0, f"Validation Failed: {v_str}", False

        if not res.get("stl_path"):
            return -10.0, "Submission failed: No STL exported.", False

        # 4. Simulation (Host side)
        try:
            stl_path = Path(self.workspace_dir) / res["stl_path"]
            template_xml = self.sim_bridge.load_template()
            injected_xml = self.sim_bridge.inject_design(template_xml, stl_path)
            sim_result = self.sim_bridge.run_simulation(injected_xml)

            reward = (
                (100.0 - (sim_result.energy * 0.1) - (sim_result.damage * 10.0))
                if sim_result.success
                else -50.0
            )
            status = "success" if sim_result.success else "failure"

            self.db.update_episode_status(
                self.episode.id,
                status=status,
                result_metrics={
                    "energy": sim_result.energy,
                    "damage": sim_result.damage,
                    "success": sim_result.success,
                    "reward": reward,
                },
            )

            report = f"Submission Result: {status.upper()}. Reward: {reward:.2f}. Energy: {sim_result.energy:.2f}"
            return reward, report, True

        except Exception as e:
            return -10.0, f"Simulation Error: {e!s}", False

    def render(self):
        return self.last_obs["last_render"]

    def close(self):
        if self.db:
            self.db.close()
