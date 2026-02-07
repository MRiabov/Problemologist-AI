import os
import re
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
        plan=state.get("plan", "No plan provided."),
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
        exec(script, local_scope)
        build_func = local_scope.get("build")
        if not build_func:
            raise AttributeError("build() function not found in script.")
            
        # 2. Run a few test builds with different seeds
        for seed in [0, 42]:
            part, mjcf = build_func(seed=seed)
            
            # 3. Geometric Validation
            if not validate(part):
                state["simulation_result"] = {
                    "valid": False,
                    "cost": 0,
                    "logs": [f"Geometric validation failed for seed {seed}"]
                }
                return state
                
            # 4. Physics Simulation
            sim_res = simulate(part)
            if not sim_res.success:
                state["simulation_result"] = {
                    "valid": False,
                    "cost": 0,
                    "logs": [f"Physics simulation failed for seed {seed}: {sim_res.summary}"]
                }
                return state
                
        # 5. Success
        state["simulation_result"] = {
            "valid": True,
            "cost": 0,
            "logs": ["Validation passed for all test seeds."]
        }
        logger.info("validator_node_complete", success=True)
        
    except Exception as e:
        logger.error("validation_node_error", error=str(e))
        state["simulation_result"] = {
            "valid": False,
            "cost": 0,
            "logs": [f"Validation error: {str(e)}"]
        }
        
    return state