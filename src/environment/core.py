import os
import time
import uuid
from typing import Any, Dict, List, Optional, Tuple, Union

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from src.compiler import geometry, mujoco_bridge
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
    ):
        super().__init__()
        self.problem_id = problem_id
        self.task_description = task_description
        self.workspace_dir = os.path.abspath(workspace_dir)
        os.makedirs(self.workspace_dir, exist_ok=True)
        tools.set_workspace_dir(self.workspace_dir)

        # Persistence
        self.db = persistence.DatabaseManager(db_url)
        self.db.create_tables()
        self.episode = None
        self.step_count = 0

        # Workbench
        self.workbench = Print3DWorkbench()

        # Simulation
        self.sim_bridge = mujoco_bridge.MujocoBridge()

        # Action Space
        # tool: 0=write, 1=edit, 2=preview, 3=search, 4=submit, 5=search_parts, 6=preview_part
        self.action_space = spaces.Dict(
            {
                "tool": spaces.Discrete(7),
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
            }
        )

        self.last_obs = {
            "code": "",
            "last_output": "",
            "last_render": "",
            "task_description": self.task_description,
            "error": "",
        }

    def reset(
        self, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None
    ) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        super().reset(seed=seed)

        # Create new episode in database
        self.episode = self.db.create_episode(self.problem_id)
        self.step_count = 0

        # Reset workspace - clear the design script
        design_path = os.path.join(self.workspace_dir, "design.py")
        with open(design_path, "w", encoding="utf-8") as f:
            f.write("")

        self.last_obs = {
            "code": "",
            "last_output": "Environment reset. Ready for new design.",
            "last_render": "",
            "task_description": self.task_description,
            "error": "",
        }

        return self.last_obs, {}

    def step(
        self, action: Dict[str, Any]
    ) -> Tuple[Dict[str, Any], float, bool, bool, Dict[str, Any]]:
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
            elif tool_name == "submit_design":
                reward, tool_output, terminated = self._submit_design()
            else:
                tool_output = f"Unknown tool index: {tool_idx}"
        except Exception as e:
            tool_output = f"Exception during tool execution: {str(e)}"
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
        script_path = os.path.join(self.workspace_dir, "design.py")
        if os.path.exists(script_path):
            with open(script_path, "r", encoding="utf-8") as f:
                self.last_obs["code"] = f.read()

        self.last_obs["last_output"] = tool_output

        return self.last_obs, reward, terminated, truncated, {}

    def _submit_design(self) -> Tuple[float, str, bool]:
        """Handles the full submission, validation, and simulation pipeline."""
        script_path = os.path.join(self.workspace_dir, "design.py")
        if not os.path.exists(script_path):
            return -10.0, "Error: No design.py found.", False

        # 1. Load and execute script to get Part
        locs = {}
        try:
            with open(script_path, "r", encoding="utf-8") as f:
                code = f.read()

            import build123d as bd

            # Execution context with common build123d symbols
            ctx = {
                "bd": bd,
                "Part": bd.Part,
                "Box": bd.Box,
                "Sphere": bd.Sphere,
                "Cylinder": bd.Cylinder,
                "Compound": bd.Compound,
                "Solid": bd.Solid,
                "Rotation": bd.Rotation,
                "Location": bd.Location,
            }
            exec(code, ctx, locs)

            part = None
            for val in locs.values():
                if isinstance(val, bd.Part):
                    part = val
                    break
                elif isinstance(val, (bd.Solid, bd.Compound, bd.Shape)):
                    # Wrap raw shapes in a Part
                    part = bd.Part()
                    part.add(val)
                    break

            if not part:
                return -10.0, "Error: No Part or Solid found in script.", False

        except Exception as e:
            return -10.0, f"Error executing script: {str(e)}", False

        # 2. WP02: Validate (Manifold/SingleBody)
        violations = self.workbench.validate(part)
        if violations:
            return -10.0, f"Validation Failed: {', '.join(map(str, violations))}", False

        # 3. WP02: Export Mesh
        stl_path = os.path.join(self.workspace_dir, "design.stl")
        try:
            geometry.export_mesh(part, stl_path)
        except Exception as e:
            return -10.0, f"Error exporting mesh: {str(e)}", False

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
            return -10.0, f"Error during simulation: {str(e)}", False

    def render(self):
        return self.last_obs["last_render"]
