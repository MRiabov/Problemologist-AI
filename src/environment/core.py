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
                # Inject environment constraints if not provided
                if "target_quantity" not in arguments:
                    arguments["target_quantity"] = self.target_quantity
                if "max_unit_cost" not in arguments:
                    arguments["max_unit_cost"] = self.max_unit_cost

                # Dispatch to runtime
                tool_output = self.runtime.dispatch(tool_name, arguments)

                # Parse output to handle persistence and termination
                try:
                    res = json.loads(tool_output)
                    reward = res.get("reward", 0.0)
                    terminated = res.get("terminated", False)
                    status = res.get("status", "unknown")
                    message = res.get("message", "")
                    metrics = res.get("metrics", {})

                    if terminated:
                        tool_output = f"{message}\n\n[TERMINATED] Reward: {reward}"
                        self.db.update_episode_status(
                            self.episode.id,
                            status=status,
                            result_metrics={**metrics, "reward": reward},
                        )
                    else:
                        tool_output = message

                except json.JSONDecodeError:
                    # Fallback if runtime didn't return JSON
                    pass
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

    def step(
        self, action: dict[str, Any]
    ) -> tuple[dict[str, Any], float, bool, bool, dict[str, Any]]:
        """
        Gym-like step method.
        action: {"tool": "tool_name", "arguments": { ... }}
        """
        tool_name = action.get("tool")
        arguments = action.get("arguments", {})

        output = self.dispatch(tool_name, arguments)

        reward = 0.0
        terminated = False
        info = {"output": output}

        try:
            res = json.loads(output)
            reward = float(res.get("reward", 0.0))
            terminated = bool(res.get("terminated", False))
            info.update(res)
        except (json.JSONDecodeError, TypeError, ValueError):
            pass

        return self.last_obs, reward, terminated, False, info

    def render(self):
        return self.last_obs["last_render"]

    def close(self):
        if self.db:
            self.db.close()
