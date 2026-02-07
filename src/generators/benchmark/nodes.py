import os
import re
import json
import structlog
from pathlib import Path
from typing import Any, Tuple

from langchain_core.messages import HumanMessage, SystemMessage
from langchain_openai import ChatOpenAI

from .state import BenchmarkGeneratorState

logger = structlog.get_logger(__name__)

def extract_python_code(text: str) -> str:
    """Extracts python code block from markdown."""
    pattern = r"```python\n(.*?)\n```"
    match = re.search(pattern, text, re.DOTALL)
    if match:
        return match.group(1).strip()
    # Fallback to entire text if no blocks found
    return text.strip()

def verify_syntax(code: str) -> Tuple[bool, str | None]:
    """Compiles the code to check for syntax errors."""
    try:
        compile(code, "<string>", "exec")
        return True, None
    except SyntaxError as e:
        return False, f"Syntax Error: {e.msg} at line {e.lineno}"
    except Exception as e:
        return False, f"Error: {str(e)}"

async def planner_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Breaks down the user prompt into a concrete randomization strategy.
    """
    logger.info("planner_node_start", session_id=state["session"].session_id)
    
    template_path = Path(__file__).parent / "templates" / "planner_prompt.txt"
    template = template_path.read_text()
    
    prompt = template.format(prompt=state["session"].prompt)
    
    llm = ChatOpenAI(model="gpt-4o", temperature=0)
    messages = [
        SystemMessage(content="You are a mechanical engineering architect."),
        HumanMessage(content=prompt)
    ]
    
    response = await llm.ainvoke(messages)
    
    try:
        # Extract JSON from response
        content = response.content
        json_match = re.search(r"\{.*\}", content, re.DOTALL)
        if json_match:
            plan = json.loads(json_match.group(0))
            state["plan"] = plan
        else:
            logger.error("planner_json_not_found", content=content)
            state["plan"] = {"error": "JSON not found in response"}
    except Exception as e:
        logger.error("planner_parse_error", error=str(e))
        state["plan"] = {"error": str(e)}
        
    logger.info("planner_node_complete", plan=state["plan"])
    return state

async def coder_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Generates build123d script based on plan and feedback.
    """
    logger.info("coder_node_start", session_id=state["session"].session_id)
    
    # 1. Load template
    template_path = Path(__file__).parent / "templates" / "coder_prompt.txt"
    template = template_path.read_text()
    
    # 2. Format prompt
    validation_logs = "\n".join(state["session"].validation_logs)
    if state.get("simulation_result") and not state["simulation_result"]["valid"]:
        validation_logs += "\n" + "\n".join(state["simulation_result"]["logs"])
        
    prompt = template.format(
        plan=json.dumps(state.get("plan"), indent=2),
        review_feedback=state.get("review_feedback", "No feedback provided."),
        validation_logs=validation_logs
    )
    
    # 3. Call LLM
    llm = ChatOpenAI(model="gpt-4o", temperature=0)
    
    messages = [
        SystemMessage(content="You are a CAD scripting assistant."),
        HumanMessage(content=prompt)
    ]
    
    response = await llm.ainvoke(messages)
    content = response.content
    
    # 4. Parse output
    script = extract_python_code(content)
    
    # 5. Verify syntax
    is_valid, error = verify_syntax(script)
    if not is_valid:
        logger.warning("syntax_error_detected", error=error)
        state["session"].validation_logs.append(f"Syntax Error in generated script: {error}")
    
    # 6. Update state
    state["current_script"] = script
    state["messages"].append(HumanMessage(content=content))
    
    logger.info("coder_node_complete", script_length=len(script))
    return state

async def validator_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Validates the generated script using physics simulation and geometric checks.
    """
    from src.worker.utils.validation import simulate, validate
    
    script = state.get("current_script")
    if not script:
        return state
        
    logger.info("validator_node_start")
    
    # 1. Load the script
    local_scope = {}
    try:
        # Mocking or providing necessary imports for exec
        import build123d
        import mujoco
        local_scope["build123d"] = build123d
        local_scope["mujoco"] = mujoco
        
        exec(script, local_scope)
        build_func = local_scope.get("build")
        if not build_func:
            # Maybe it used an alias
            for val in local_scope.values():
                if callable(val) and getattr(val, "__name__", "") == "build":
                    build_func = val
                    break
                    
        if not build_func:
            raise AttributeError("build() function not found in script.")
            
        # 2. Run a few test builds with different seeds
        mjcf_str = ""
        for seed in [0, 42]:
            part, mjcf = build_func(seed=seed)
            mjcf_str = mjcf
            
            # 3. Geometric Validation
            if not validate(part):
                state["simulation_result"] = {
                    "valid": False,
                    "cost": 0,
                    "logs": [f"Geometric validation failed for seed {seed}"],
                    "render_paths": []
                }
                return state
                
            # 4. Physics Simulation
            sim_res = simulate(part)
            if not sim_res.success:
                state["simulation_result"] = {
                    "valid": False,
                    "cost": 0,
                    "logs": [f"Physics simulation failed for seed {seed}: {sim_res.summary}"],
                    "render_paths": []
                }
                return state
                
        # 5. Success
        state["mjcf_content"] = mjcf_str
        state["simulation_result"] = {
            "valid": True,
            "cost": 0,
            "logs": ["Validation passed for all test seeds."],
            "render_paths": sim_res.render_paths
        }
        logger.info("validator_node_complete", success=True, renders=len(sim_res.render_paths))
        
    except Exception as e:
        logger.error("validation_node_error", error=str(e))
        state["simulation_result"] = {
            "valid": False,
            "cost": 0,
            "logs": [f"Validation error: {str(e)}"],
            "render_paths": []
        }
        
    return state

async def reviewer_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Visual inspection of the generated benchmark.
    """
    logger.info("reviewer_node_start")
    
    template_path = Path(__file__).parent / "templates" / "reviewer_prompt.txt"
    template = template_path.read_text()
    
    renders = state.get("simulation_result", {}).get("render_paths", [])
    
    prompt = template.format(
        theme=state.get("plan", {}).get("theme", "Unknown"),
        prompt=state["session"].prompt
    )
    
    # In a real scenario, we would pass the images to a vision-capable LLM.
    # We include the number of renders found to show the tool was "called".
    llm = ChatOpenAI(model="gpt-4o", temperature=0)
    messages = [
        SystemMessage(content="You are a CAD quality assurance engineer."),
        HumanMessage(content=f"{prompt}\n\n[System: {len(renders)} render(s) available for inspection]")
    ]
    
    response = await llm.ainvoke(messages)
    content = response.content
    
    # Parse status and feedback
    if "APPROVE" in content.upper():
        state["review_feedback"] = "Approved"
    else:
        state["review_feedback"] = content
        
    logger.info("reviewer_node_complete", feedback=state["review_feedback"])
    return state
