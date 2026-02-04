from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from langchain_core.messages import HumanMessage
from src.agent.graph.nodes.planner import planner_node
from src.generators.benchmark.agent import DEFAULT_RUNTIME_CONFIG, generator_agent
from src.generators.benchmark.renderer import render_scenario
from src.generators.benchmark.manager import execute_build
from src.generators.benchmark.validator import validate_mjcf
import tempfile
from pathlib import Path
import re
import shutil
import asyncio

router = APIRouter(prefix="/benchmark", tags=["benchmark"])

class PlanRequest(BaseModel):
    intent: str

class PlanResponse(BaseModel):
    plan: str
    reasoning: str

@router.post("/plan", response_model=PlanResponse)
async def generate_plan(req: PlanRequest):
    input_state = {
        "messages": [HumanMessage(content=req.intent)],
        "step_count": 0,
        "runtime_config": DEFAULT_RUNTIME_CONFIG,
    }
    try:
        result = await planner_node(input_state)
        return {"plan": result["plan"], "reasoning": "Plan generated via DeepAgents planner."}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

class GenerateRequest(BaseModel):
    intent: str
    plan: str

class GenerateResponse(BaseModel):
    code: str
    mjcf: str
    renders: list[str]
    logs: list[str]
    success: bool
    error: str | None

@router.post("/generate", response_model=GenerateResponse)
async def generate_cad(req: GenerateRequest):
    input_state = {
        "messages": [HumanMessage(content=req.intent)],
        "plan": req.plan,
        "step_count": 0,
        "runtime_config": DEFAULT_RUNTIME_CONFIG,
    }

    logs = []
    final_state = None

    try:
        async for event in generator_agent.astream(input_state, config={"recursion_limit": 200}, stream_mode="updates"):
             for node_name, updates in event.items():
                 logs.append(f"Node {node_name} executed.")

                 # Merge state
                 if final_state is None:
                     final_state = input_state.copy()

                 if "messages" in updates:
                     if "messages" not in final_state: final_state["messages"] = []
                     final_state["messages"].extend(updates["messages"])

                 for k, v in updates.items():
                     if k != "messages":
                         final_state[k] = v
    except Exception as e:
         return {
             "code": "", "mjcf": "", "renders": [],
             "logs": logs, "success": False, "error": str(e)
         }

    if not final_state:
        return {"code": "", "mjcf": "", "renders": [], "logs": logs, "success": False, "error": "No state returned"}

    # Extraction Logic
    messages = final_state.get("messages", [])
    last_mjcf = ""
    last_code = ""

    # Extract MJCF
    for msg in reversed(messages):
        content_str = str(msg.content)
        if "---FULL_MJCF_START---" in content_str:
            match = re.search(
                r"---FULL_MJCF_START---\n(.*?)\n---FULL_MJCF_END---",
                content_str,
                re.DOTALL,
            )
            if match:
                last_mjcf = match.group(1)
                break

    # Extract Code
    # 1. From tool calls
    for msg in reversed(messages):
        if hasattr(msg, "tool_calls"):
            for tc in msg.tool_calls:
                if tc["name"] == "validate_benchmark_model":
                    last_code = tc["args"].get("code", "")
                    break
                if tc["name"] == "write_script":
                    last_code = tc["args"].get("content", "")
                    break
        if last_code:
            break

    renders = []
    if last_mjcf:
        try:
            with tempfile.TemporaryDirectory() as tmpdir:
                prefix = Path(tmpdir) / "preview"
                # Mock render if mujoco not available in this env
                # But try to call real one
                try:
                    render_paths = render_scenario(last_mjcf, str(prefix))
                except Exception as render_err:
                    logs.append(f"Render failed (likely headless): {render_err}")
                    render_paths = []

                # Ensure static dir exists
                # We need to serve this. Let's put it in a persistent place.
                static_dir = Path("static/renders")
                static_dir.mkdir(parents=True, exist_ok=True)

                valid_paths = []
                for p in render_paths:
                    dest = static_dir / Path(p).name
                    shutil.copy(str(p), str(dest))
                    valid_paths.append(f"/static/renders/{dest.name}")
                renders = valid_paths
        except Exception as e:
            logs.append(f"Render processing failed: {e}")

    return {
        "code": last_code,
        "mjcf": last_mjcf,
        "renders": renders,
        "logs": logs,
        "success": bool(last_mjcf),
        "error": None if last_mjcf else "Failed to generate MJCF"
    }
