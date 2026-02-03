import json
import time
from pathlib import Path
from typing import Any

import gymnasium as gym
from gymnasium import spaces

from src.compiler import mujoco_bridge
from src.environment import persistence
from src.environment.runtime import ToolRuntime


class CADEnv(gym.Env):
    """
    Agentic CAD Environment.
    Implements a Gymnasium-compatible interface for LLM-based mechanical engineering agents.
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}

    def __init__(
        self,
        problem_id: str = "default_problem",
        task_description: str = "Design a part that satisfies geometric constraints.",
        db_url: str = "sqlite:///history.db",
        workspace_dir: str = "workspace",
        max_unit_cost: float = float("inf"),
        target_quantity: int = 1,
    ):
        super().__init__()
        self.problem_id = problem_id
        self.task_description = task_description
        self.max_unit_cost = max_unit_cost
        self.target_quantity = target_quantity

        # Initialize Runtime
        self.runtime = ToolRuntime(
            workspace_dir, None
        )  # DB pass later if needed inside runtime

        # Persistence
        self.db = persistence.DatabaseManager(db_url)
        self.db.create_tables()
        self.runtime.db = self.db  # Inject DB

        self.episode = None
        self.step_count = 0

        # Simulation
        self.sim_bridge = mujoco_bridge.MujocoBridge()

        # Action Space
        # tool: String name of the tool (e.g., "write_file", "edit_file")
        # arguments: JSON string of arguments
        self.action_space = spaces.Dict(
            {
                "tool": spaces.Text(min_length=0, max_length=50),
                "arguments": spaces.Text(min_length=0, max_length=100000),
            }
        )

        # Observation Space
        self.observation_space = spaces.Dict(
            {
                "code": spaces.Text(min_length=0, max_length=100000),
                "last_output": spaces.Text(min_length=0, max_length=100000),
                "last_render": spaces.Text(min_length=0, max_length=1000),
                "task_description": spaces.Text(min_length=0, max_length=10000),
                "error": spaces.Text(min_length=0, max_length=10000),
                "budget_info": spaces.Text(min_length=0, max_length=1000),
            }
        )

        self.last_obs = {
            "code": "",
            "last_output": "",
            "last_render": "",
            "task_description": self.task_description,
            "error": "",
            "budget_info": f"Target Quantity: {self.target_quantity}, Max Unit Cost: ${self.max_unit_cost}",
        }

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        super().reset(seed=seed)

        # Create new episode in database
        self.episode = self.db.create_episode(self.problem_id)
        self.step_count = 0

        # Reset workspace - clear the design script
        design_path = Path(self.runtime.workspace_dir) / "design.py"
        design_path.write_text("", encoding="utf-8")

        # Stop any existing session? Maybe not needed as session is persistent across resets usually,
        # but logically we might want a fresh start. For now keeping session alive.

        self.last_obs = {
            "code": "",
            "last_output": "Environment reset. Ready for new design.",
            "last_render": "",
            "task_description": self.task_description,
            "error": "",
            "budget_info": f"Target Quantity: {self.target_quantity}, Max Unit Cost: ${self.max_unit_cost}",
        }

        return self.last_obs, {}

    def step(
        self, action: dict[str, Any], agent_role: str | None = None
    ) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        self.step_count += 1
        start_time = time.time()

        tool_name = action.get("tool", "unknown")
        raw_arguments = action.get("arguments", "{}")

        # 1. Parse Arguments
        arguments = {}
        try:
            if isinstance(raw_arguments, str) and raw_arguments.strip().startswith("{"):
                arguments = json.loads(raw_arguments)
            elif isinstance(raw_arguments, dict):
                arguments = raw_arguments
            else:
                arguments = {"value": raw_arguments}
        except json.JSONDecodeError:
            arguments = {"value": raw_arguments}

        tool_output = ""
        reward = 0.0
        terminated = False
        truncated = False

        try:
            # 2. Dispatch Tool
            if tool_name == "submit_design":
                # Special handling for submit_design which is complex and specific to Env logic (simulation, reward)
                # Although ToolRuntime could have submit_design, the reward calculation is here.
                # So we keep _submit_design here, but maybe move the sandbox execution part to Runtime?
                # _submit_design uses sandbox.run_script.
                # Let's keep it here for now as it orchestrates simulation too.
                reward, tool_output, terminated = self._submit_design(arguments)
            else:
                # Dispatch to Runtime
                tool_output = self.runtime.dispatch(tool_name, arguments)

                # Side effects checking
                if (
                    tool_name == "preview_design"
                    and "Preview generated:" in tool_output
                ):
                    try:
                        self.last_obs["last_render"] = tool_output.split(
                            "Preview generated:"
                        )[1].strip()
                    except:
                        pass

        except Exception as e:
            tool_output = f"Exception during tool execution: {e!s}"
            self.last_obs["error"] = str(e)

        duration_ms = int((time.time() - start_time) * 1000)

        # Persistence
        db_step = self.db.log_step(
            episode_id=self.episode.id,
            sequence_index=self.step_count,
            tool_name=tool_name,
            tool_input=json.dumps(arguments)
            if isinstance(arguments, dict)
            else str(arguments),
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

        # Update observation
        script_path = Path(self.runtime.workspace_dir) / "design.py"
        if script_path.exists():
            self.last_obs["code"] = script_path.read_text(encoding="utf-8")

        self.last_obs["last_output"] = tool_output

        return self.last_obs, reward, terminated, truncated, {}

    def log_message(
        self,
        content: str,
        type: str = "thought",
        agent_role: str | None = None,
        metadata: dict[str, Any] | None = None,
    ):
        """Logs a non-action message (thought, handoff, user) to the database."""
        self.step_count += 1
        self.db.log_step(
            episode_id=self.episode.id,
            sequence_index=self.step_count,
            type=type,
            content=content,
            agent_role=agent_role,
            metadata_json=metadata,
        )

    def _submit_design(
        self, arguments: str | dict[str, Any] = ""
    ) -> tuple[float, str, bool]:
        """Handles the full submission, validation, and simulation pipeline."""
        script_path = Path(self.runtime.workspace_dir) / "design.py"
        if not script_path.exists():
            return -10.0, "Error: No design.py found.", False

        # Parse arguments for force submit
        force_submit = False

        if isinstance(arguments, dict):
            force_submit = arguments.get("force_submit", False)
            if isinstance(force_submit, str):
                force_submit = force_submit.lower() == "true"
        elif isinstance(arguments, str):
            if "force_submit=True" in arguments:
                force_submit = True

        # Reuse ToolRuntime's manufacturability check which is now secure!
        # But wait, submit_design does MORE than check_manufacturability.
        # It generates STL and runs simulation.
        # And it enforces budget.

        # We need a secure way to generate STL.
        # The runtime has check_manufacturability, but maybe we need a generate_stl method there too?
        # Or we can treat submit_design as a special "super-tool" in Runtime?

        # For now, let's replicate the secure runner pattern here using self.runtime.sandbox

        runner_filename = "submit_runner.py"
        runner_path = Path(self.runtime.workspace_dir) / runner_filename
        stl_filename = "design.stl"

        # This runner is similar to runtime.check_manufacturability but also exports STL
        runner_script = f"""
import sys
import json
import hashlib
from pathlib import Path
import build123d as bd
from src.workbenches.print_3d import Print3DWorkbench
from src.workbenches.cnc import CNCWorkbench
from src.workbenches.injection_molding import InjectionMoldingWorkbench
from src.compiler import geometry

sys.path.append("/workspace")

result = {{
    "success": False,
    "error": None,
    "violations": [],
    "stl_path": None,
    "total_cost": 0.0,
    "unit_cost": 0.0
}}

try:
    locs = {{}}
    with Path("/workspace/design.py").open("r", encoding="utf-8") as f:
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

    if not export_obj:
        result["error"] = "No Part or Solid found in script."
    else:
        workbenches = {{
            "print_3d": Print3DWorkbench(),
            "cnc": CNCWorkbench(),
            "injection_molding": InjectionMoldingWorkbench()
        }}
        
        # Simplified costing/validation (similar to Runtime)
        default_q = {self.target_quantity}
        
        def parse_label(label):
            data = {{"quantity": default_q, "process": "print_3d"}}
            if not label: return data
            for p in str(label).split("|"):
                if ":" in p:
                    k, v = p.split(":", 1)
                    if k == "quantity": 
                        try: data["quantity"] = int(v)
                        except: pass
                    elif k == "process": data["process"] = v
            return data

        all_solids = []
        if isinstance(export_obj, bd.Compound):
            all_solids = list(export_obj.solids())
        else:
            all_solids = [export_obj]

        total_cost = 0.0
        reuse_ctx = {{}}
        all_violations = []

        for solid in all_solids:
            ldata = parse_label(getattr(solid, "label", ""))
            wb = workbenches.get(ldata["process"], workbenches["print_3d"])
            all_violations.extend(wb.validate(solid))
            total_cost += wb.calculate_cost(solid, ldata["quantity"], context=reuse_ctx)

        result["total_cost"] = total_cost
        result["unit_cost"] = total_cost / default_q if default_q > 0 else 0.0
        result["violations"] = [str(v) for v in all_violations]

        if not all_violations:
            stl_path = f"/workspace/{{'{stl_filename}'}}"
            geometry.export_mesh(export_obj, stl_path)
            result["success"] = True
            result["stl_path"] = "{stl_filename}"

except Exception as e:
    result["error"] = str(e)

print(f"SUBMIT_RESULT:{{json.dumps(result)}}")
"""

        try:
            runner_path.write_text(runner_script, encoding="utf-8")

            # Run in sandbox via Runtime
            stdout, stderr, returncode = self.runtime.sandbox.run_script(
                runner_filename,
                session_id=self.runtime.active_session_id,  # Use active session!
                mount_src=True,
            )

            if runner_path.exists():
                runner_path.unlink()

            if "SUBMIT_RESULT:" not in stdout:
                return -10.0, f"Sandbox execution failed: {stderr}", False

            result_line = [
                line for line in stdout.split("\n") if "SUBMIT_RESULT:" in line
            ][0]
            res_data = json.loads(result_line.split("SUBMIT_RESULT:")[1])

            if res_data["error"]:
                return -10.0, f"Error processing design: {res_data['error']}", False

            if res_data["unit_cost"] > self.max_unit_cost and not force_submit:
                return (
                    -20.0,
                    f"REJECTED: Unit cost ${res_data['unit_cost']:.2f} exceeds budget ${self.max_unit_cost:.2f}.",
                    False,
                )

            if res_data["violations"]:
                return (
                    -10.0,
                    f"Validation Failed: {', '.join(res_data['violations'])}",
                    False,
                )

            if not res_data["success"]:
                return -10.0, "Submission processing failed.", False

            stl_path = Path(self.runtime.workspace_dir) / stl_filename

        except Exception as e:
            return -10.0, f"Error during sandboxed submission: {e!s}", False

        # Simulation (Host side for now as it needs MujocoBridge which might not be in container yet?
        # Actually SimBridge usually runs on host using compiled binary or Python bindings)
        try:
            template_xml = self.sim_bridge.load_template()
            injected_xml = self.sim_bridge.inject_design(template_xml, stl_path)
            sim_result = self.sim_bridge.run_simulation(injected_xml)

            if sim_result.success:
                reward = 100.0 - (sim_result.energy * 0.1) - (sim_result.damage * 10.0)
                status = "success"
            else:
                reward = -50.0
                status = "failure"

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

            report = (
                f"Submission Result: {status.upper()}. Reward: {reward:.2f}. "
                f"Energy: {sim_result.energy:.2f}, Damage: {sim_result.damage:.2f}"
            )
            return reward, report, True

        except Exception as e:
            return -10.0, f"Error during simulation: {e!s}", False

    def render(self):
        return self.last_obs["last_render"]

    def close(self):
        """Cleanup resources."""
        if self.db:
            self.db.close()
        if hasattr(self.runtime, "stop_session"):
            # Optional: stop sandbox session if strictly bound to env checking
            pass
        super().close()
