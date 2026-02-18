import asyncio
import structlog
from typing import Any, Callable, Dict, Optional
from controller.clients.worker import WorkerClient

logger = structlog.get_logger(__name__)


class WorkerInterpreter:
    """
    A DSPy CodeInterpreter that executes code on the remote worker.
    Uses the project's 'well formed remote execution environment'.
    """

    def __init__(self, worker_client: WorkerClient, session_id: str):
        self.worker_client = worker_client
        self.session_id = session_id

    def __call__(self, code: str) -> str:
        """Execute code on the worker and return the output."""
        logger.debug("worker_interpreter_execute_start", code_len=len(code))

        # DSPy CodeAct expects a synchronous call.
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

        if loop.is_running():
            import nest_asyncio

            nest_asyncio.apply()

        result = loop.run_until_complete(self._execute_remote(code))

        output = f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        if result.exit_code != 0:
            output += f"\nExit Code: {result.exit_code}"

        logger.debug("worker_interpreter_execute_complete", exit_code=result.exit_code)
        return output

    async def _execute_remote(self, code: str):
        """Internal async execution call."""
        return await self.worker_client.execute_python(code)

    def shutdown(self):
        """Cleanup if needed."""
        pass


def evaluate_formula(formula: str, context: dict) -> float:
    """Safely eval a math formula with a provided context."""
    try:
        allowed_names = {
            "max": max,
            "min": min,
            "abs": abs,
            "pow": pow,
            "round": round,
            "n_valid_candidates": context.get("n_valid_candidates", 0),
            "n_returned_candidates": context.get("n_returned_candidates", 1),
            "n_queries": context.get("n_queries", 0),
            "delta_success_rate": context.get("delta_success_rate", 0.0),
        }
        allowed_names.update(context)
        return float(eval(formula, {"__builtins__": {}}, allowed_names))
    except Exception as e:
        logger.error("formula_eval_failed", formula=formula, error=str(e))
        return 0.0


def cad_simulation_metric(
    gold, prediction, trace=None, pred_name=None, pred_trace=None
) -> Any:
    """
    Unified DSPy metric for CAD simulation performance.
    Returns a dspy.Prediction with 'score' and 'feedback' for GEPA.
    """
    from dspy import Prediction
    from controller.agent.reward import load_reward_config

    agent_name = getattr(gold, "agent_name", "cad_engineer")
    full_cfg = load_reward_config()

    # Find the agent config in the hierarchy
    cfg = None
    for category in ["engineer", "benchmark_generator", "shared"]:
        cat_cfg = getattr(full_cfg, category, {})
        if agent_name in cat_cfg:
            cfg = cat_cfg[agent_name]
            break

    if not cfg:
        logger.warning("reward_config_agent_not_found", agent_name=agent_name)
        return Prediction(
            score=0.02, feedback=f"Agent '{agent_name}' not found in reward_config."
        )

    score = 0.0
    feedback_parts = []
    context = {"prediction": prediction, "gold": gold}

    # Map attributes for formula evaluation
    mapping = {
        "actual_cost": ["actual_cost"],
        "actual_weight": ["actual_weight"],
        "min_distance_achieved": ["min_distance_to_goal", "min_distance_achieved"],
        "initial_distance": ["initial_distance"],
        "estimated_cost": ["estimated_cost"],
        "estimated_weight": ["estimated_weight"],
        "total_power": ["total_power"],
        "n_valid_candidates": ["n_valid_candidates"],
        "n_returned_candidates": ["n_returned_candidates"],
        "n_queries": ["n_queries"],
        "delta_success_rate": ["delta_success_rate"],
    }

    for obj in [prediction, gold]:
        for target, sources in mapping.items():
            for src in sources:
                if hasattr(obj, src):
                    context[target] = getattr(obj, src)
                    break
        if hasattr(obj, "objectives"):
            obj_yaml = obj.objectives
            for k in ["max_unit_cost", "max_weight", "max_power"]:
                if hasattr(obj_yaml, k):
                    context[k] = getattr(obj_yaml, k)

    error_msg = getattr(prediction, "error", None)

    for milestone_name, m in cfg.milestones.items():
        m_score = 0.0
        details = ""

        aliases = {
            "script_compiles": ["script_compiled", "compiled"],
            "simulation_result": ["simulation_success", "simulation_ran"],
            "cad_geometry_valid": ["geometry_valid", "cad_geometry_valid"],
            "reviewer_accepted": ["accepted", "reviewer_accepted"],
            "manufacturability_valid": ["manufacturability_valid"],
            "parts_within_build_zone": ["parts_within_build_zone"],
        }
        candidates = [milestone_name, *aliases.get(milestone_name, [])]
        is_triggered = any(getattr(prediction, c, False) for c in candidates)

        if milestone_name == "script_compiles":
            if not is_triggered:
                fb = m.failure_feedback or "Script did not compile."
                if error_msg:
                    fb += f" Error: {error_msg}"
                return Prediction(score=m.minimum_score or 0.02, feedback=fb)
            m_score = 1.0
            details = m.success_feedback or "Script compiled successfully."
        elif milestone_name == "cad_geometry_valid":
            if not is_triggered:
                fb = m.failure_feedback or "CAD geometry is invalid or missing."
                return Prediction(score=score, feedback=fb)
            m_score = 1.0
            details = m.success_feedback or "CAD geometry is valid."
        elif m.binary:
            m_score = 1.0 if is_triggered else 0.0
            details = (
                (m.success_feedback if is_triggered else m.failure_feedback)
                or f"{milestone_name.replace('_', ' ').capitalize()} status: {is_triggered}"
            )
        elif m.partial:
            # Handle partial credit and select feedback
            is_sim_ran = any(
                getattr(prediction, c, False)
                for c in ["simulation_ran", "simulation_success"]
            )
            if milestone_name == "simulation_result" and not is_sim_ran:
                m_score = 0.0
                details = m.failure_feedback or "Simulation did not run."
            elif milestone_name in ["simulation_result", "decision_correct"]:
                if getattr(prediction, "simulation_success", False) or getattr(
                    prediction, "decision_correct", False
                ):
                    m_score = 1.0
                    details = m.success_feedback or "Full success achieved."
                elif m.failure_formula:
                    m_score = evaluate_formula(m.failure_formula, context) / m.weight
                    details = (
                        m.failure_feedback or "Partial credit awarded."
                    ) + f" (Progress: {m_score:.2f})"
            else:
                f = m.penalty_formula or m.formula
                if f:
                    m_score = evaluate_formula(f, context)
                    details = (
                        m.success_feedback if m_score >= 0.9 else m.failure_feedback
                    ) or f"{milestone_name} score: {m_score:.2f}"
                else:
                    m_score = 1.0 if is_triggered else 0.0
                    details = (
                        m.success_feedback if is_triggered else m.failure_feedback
                    ) or f"{milestone_name} result: {is_triggered}"
        else:
            m_score = 1.0 if is_triggered else 0.0
            details = (
                m.success_feedback if is_triggered else m.failure_feedback
            ) or f"{milestone_name} result: {is_triggered}"

        score += m.weight * m_score
        feedback_parts.append(details)

    final_fb = " | ".join(filter(None, feedback_parts))

    # Append specific reviewer feedback if available and not already covered
    rev_fb = getattr(prediction, "reviewer_feedback", None)
    if rev_fb and "Reviewer Rejection" not in (error_msg or ""):
        final_fb += f" | Reviewer Feedback: {rev_fb}"

    if error_msg:
        final_fb = f"Error: {error_msg} | {final_fb}"

    return Prediction(score=min(score, 1.0), feedback=final_fb)


def map_events_to_prediction(events: list[dict], _objectives: Any = None) -> dict:
    """Translates worker events and objectives into a flatter dict."""
    pred = {
        "script_compiled": False,
        "cad_geometry_valid": False,
        "manufacturability_valid": False,
        "parts_within_build_zone": False,
        "simulation_success": False,
        "simulation_ran": False,
        "actual_cost": 0.0,
        "actual_weight": 0.0,
        "min_distance_to_goal": 1.0,
        "initial_distance": 1.0,
        "error": None,
    }

    for event in events:
        etype = event.get("type")
        data = event.get("data", {})

        if etype == "simulation_start":
            pred["simulation_ran"] = True
        elif etype == "simulation_success":
            pred["simulation_success"] = True
        elif etype == "cad_build_success":
            pred["script_compiled"] = True
            pred["cad_geometry_valid"] = True
        elif etype == "validation_success":
            pred["manufacturability_valid"] = True
            pred["parts_within_build_zone"] = True
            pred["actual_cost"] = data.get("total_cost", 0.0)
            pred["actual_weight"] = data.get("total_weight", 0.0)
        elif etype == "simulation_metrics":
            pred["min_distance_to_goal"] = data.get("min_distance", 1.0)
            pred["initial_distance"] = data.get("initial_distance", 1.0)
        elif etype in ["simulation_error", "execution_error", "cad_build_error"]:
            pred["error"] = data.get("error") or data.get("message") or str(data)
        elif etype == "review_decision":
            pred["reviewer_decision"] = data.get("decision")
            pred["reviewer_feedback"] = data.get("reason")

            if data.get("decision") == "approve":
                pred["reviewer_accepted"] = True

            # If rejected, treat it as a 'soft' error for the metric unless overriden
            if data.get("decision") in ["reject_plan", "reject_code"]:
                # Append to error if existing, or set it
                existing_err = pred.get("error")
                new_err = f"Reviewer Rejection: {data.get('reason')}"
                pred["error"] = (
                    f"{existing_err} | {new_err}" if existing_err else new_err
                )

    return pred
