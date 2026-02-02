import json
import time
from pathlib import Path
from typing import Any

import gymnasium as gym
from gymnasium import spaces

from src.compiler import mujoco_bridge
from src.environment import persistence, tools
from src.workbenches.print_3d import Print3DWorkbench


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
        self.workspace_dir = str(Path(workspace_dir).resolve())
        Path(self.workspace_dir).mkdir(parents=True, exist_ok=True)
        tools.set_workspace_dir(self.workspace_dir)

        # Persistence
        self.db = persistence.DatabaseManager(db_url)
        self.db.create_tables()
        self.episode = None
        self.step_count = 0

        # Workbench (Default, can be overridden by part labels)
        self.workbench = Print3DWorkbench()

        # Simulation
        self.sim_bridge = mujoco_bridge.MujocoBridge()

        # Action Space
        # tool: 0=write, 1=edit, 2=preview, 3=search, 4=submit, 5=search_parts, 6=preview_part, 7=manufacturability
        self.action_space = spaces.Dict(
            {
                "tool": spaces.Discrete(8),
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
        design_path = Path(self.workspace_dir) / "design.py"
        design_path.write_text("", encoding="utf-8")

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
        self, action: dict[str, Any]
    ) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        self.step_count += 1
        start_time = time.time()

        tool_idx = action.get("tool")
        arguments = action.get("arguments", "")

        tool_map = {
            0: "write_script",
            1: "edit_script",
            2: "preview_design",
            3: "search_docs",
            4: "submit_design",
            5: "search_parts",
            6: "preview_part",
            7: "check_manufacturability",
        }
        tool_name = tool_map.get(tool_idx, "unknown")

        tool_output = ""
        reward = 0.0
        terminated = False
        truncated = False

        try:
            if tool_name == "write_script":
                tool_output = tools.write_script(arguments)
            elif tool_name == "edit_script":
                if "|||" in arguments:
                    find, replace = arguments.split("|||", 1)
                    tool_output = tools.edit_script("design.py", find, replace)
                else:
                    tool_output = "Error: edit_script requires 'find|||replace' format."
            elif tool_name == "preview_design":
                tool_output = tools.preview_design()
                if "Preview generated:" in tool_output:
                    self.last_obs["last_render"] = tool_output.split(
                        "Preview generated:"
                    )[1].strip()
            elif tool_name == "search_docs":
                tool_output = tools.search_docs(arguments)
            elif tool_name == "search_parts":
                tool_output = tools.search_parts(arguments)
            elif tool_name == "preview_part":
                tool_output = tools.preview_part(arguments)
            elif tool_name == "check_manufacturability":
                if "|||" in arguments:
                    process, quantity_str = arguments.split("|||", 1)
                    try:
                        quantity = int(quantity_str)
                    except ValueError:
                        quantity = self.target_quantity
                else:
                    process = arguments if arguments else "cnc"
                    quantity = self.target_quantity
                tool_output = str(tools.check_manufacturability("design.py", process, quantity))
            elif tool_name == "submit_design":
                reward, tool_output, terminated = self._submit_design(arguments)
            else:
                tool_output = f"Unknown tool index: {tool_idx}"
        except Exception as e:
            tool_output = f"Exception during tool execution: {e!s}"
            self.last_obs["error"] = str(e)

        duration_ms = int((time.time() - start_time) * 1000)

        # Persistence
        db_step = self.db.log_step(
            episode_id=self.episode.id,
            sequence_index=self.step_count,
            tool_name=tool_name,
            tool_input=arguments,
            tool_output=tool_output,
            duration_ms=duration_ms,
        )

        if tool_name == "preview_design" and self.last_obs["last_render"]:
            self.db.save_artifact(
                step_id=db_step.id,
                artifact_type="image",
                file_path=self.last_obs["last_render"],
            )

        # Update observation
        script_path = Path(self.workspace_dir) / "design.py"
        if script_path.exists():
            self.last_obs["code"] = script_path.read_text(encoding="utf-8")

        self.last_obs["last_output"] = tool_output

        return self.last_obs, reward, terminated, truncated, {}

    def _submit_design(self, arguments: str = "") -> tuple[float, str, bool]:
        """Handles the full submission, validation, and simulation pipeline."""
        script_path = Path(self.workspace_dir) / "design.py"
        if not script_path.exists():
            return -10.0, "Error: No design.py found.", False

        # Parse arguments for force submit: "force_submit=True|||reason:..."
        force_submit = False
        justification = ""
        if "force_submit=True" in arguments:
            force_submit = True
            if "|||" in arguments:
                _, justification = arguments.split("|||", 1)

        # 1. Process design.py in Sandbox
        runner_filename = "submit_runner.py"
        runner_path = Path(self.workspace_dir) / runner_filename
        stl_filename = "design.stl"
        stl_path = Path(self.workspace_dir) / stl_filename

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

# Add workspace to path
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
    # 1. Load design.py
    locs = {{}}
    ctx = {{
        "bd": bd,
        "Part": bd.Part,
        "Box": bd.Box,
        "Sphere": bd.Sphere,
        "Cylinder": bd.Cylinder,
        "Compound": bd.Compound,
        "Solid": bd.Solid,
        "Rotation": bd.Rotation,
        "Location": bd.Location,
    }}
    
    with Path("/workspace/design.py").open("r", encoding="utf-8") as f:
        code = f.read()
    exec(code, ctx, locs)
    
    # 2. Extract Part
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
        # 3. Cost Analysis & Validation
        workbenches = {{
            "print_3d": Print3DWorkbench(),
            "cnc": CNCWorkbench(),
            "injection_molding": InjectionMoldingWorkbench()
        }}
        
        default_q = {self.target_quantity}
        
        def parse_label(label):
            data = {{"quantity": default_q, "process": "print_3d"}}
            if not label: return data
            for p in str(label).split("|"):
                if ":" in p:
                    k, v = p.split(":", 1)
                    if k == "quantity": data["quantity"] = int(v)
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
            # 4. Export Mesh
            stl_path = f"/workspace/{{'stl_filename'}}"
            geometry.export_mesh(export_obj, stl_path)
            result["success"] = True
            result["stl_path"] = "{stl_filename}"

except Exception as e:
    result["error"] = str(e)

print(f"SUBMIT_RESULT:{{json.dumps(result)}}")
"""

        try:
            # Write runner
            runner_path.write_text(runner_script, encoding="utf-8")

            # Run in sandbox
            stdout, stderr, returncode = tools._SANDBOX.run_script(
                runner_filename,
                mount_src=True,
                timeout=60,
            )

            # Clean up runner
            if runner_path.exists():
                runner_path.unlink()

            if "SUBMIT_RESULT:" not in stdout:
                return -10.0, f"Sandbox execution failed: {stderr}", False

            # Parse result
            result_line = [
                line for line in stdout.split("\n") if "SUBMIT_RESULT:" in line
            ][0]
            res_data = json.loads(result_line.split("SUBMIT_RESULT:")[1])

            if res_data["error"]:
                return -10.0, f"Error processing design: {res_data['error']}", False

            # HARD BUDGET ENFORCEMENT
            if res_data["unit_cost"] > self.max_unit_cost and not force_submit:
                return (
                    -20.0,
                    f"REJECTED: Unit cost ${res_data['unit_cost']:.2f} exceeds budget ${self.max_unit_cost:.2f}. "
                    "You must optimize the design. If you believe no further improvement is possible, "
                    "submit with 'force_submit=True|||reason:<your_justification>'.",
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

        except Exception as e:
            return -10.0, f"Error during sandboxed submission: {e!s}", False

        # 4. WP03: Simulation
        try:
            template_xml = self.sim_bridge.load_template()
            injected_xml = self.sim_bridge.inject_design(template_xml, stl_path)
            sim_result = self.sim_bridge.run_simulation(injected_xml)

            # 5. Calculate Reward
            # Success (100) - Energy*0.1 - Damage*10
            if sim_result.success:
                reward = 100.0 - (sim_result.energy * 0.1) - (sim_result.damage * 10.0)
                status = "success"
            else:
                reward = -50.0  # Penalty for physical failure
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
