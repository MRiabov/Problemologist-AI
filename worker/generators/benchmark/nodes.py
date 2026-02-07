import json
import re
from pathlib import Path

import structlog
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


def verify_syntax(code: str) -> tuple[bool, str | None]:
    """Compiles the code to check for syntax errors."""
    try:
        compile(code, "<string>", "exec")
        return True, None
    except SyntaxError as e:
        return False, f"Syntax Error: {e.msg} at line {e.lineno}"
    except Exception as e:
        return False, f"Error: {e!s}"


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
        HumanMessage(content=prompt),
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
        validation_logs=validation_logs,
    )

    # 3. Call LLM
    llm = ChatOpenAI(model="gpt-4o", temperature=0)

    messages = [
        SystemMessage(content="You are a CAD scripting assistant."),
        HumanMessage(content=prompt),
    ]

    response = await llm.ainvoke(messages)
    content = response.content

    # 4. Parse output
    script = extract_python_code(content)

    # 5. Verify syntax
    is_valid, error = verify_syntax(script)
    if not is_valid:
        logger.warning("syntax_error_detected", error=error)
        state["session"].validation_logs.append(
            f"Syntax Error in generated script: {error}"
        )

    # 6. Update state
    state["current_script"] = script
    state["messages"].append(HumanMessage(content=content))

    logger.info("coder_node_complete", script_length=len(script))
    return state


async def validator_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Validates the generated script using physics simulation and geometric checks.
    """
    from worker.utils.validation import simulate, validate

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
                    "render_paths": [],
                }
                return state

            # 4. Physics Simulation
            sim_res = simulate(part)
            if not sim_res.success:
                state["simulation_result"] = {
                    "valid": False,
                    "cost": 0,
                    "logs": [
                        f"Physics simulation failed for seed {seed}: {sim_res.summary}"
                    ],
                    "render_paths": [],
                }
                return state

        # 5. Success
        state["mjcf_content"] = mjcf_str
        state["simulation_result"] = {
            "valid": True,
            "cost": 0,
            "logs": ["Validation passed for all test seeds."],
            "render_paths": sim_res.render_paths,
        }
        logger.info(
            "validator_node_complete", success=True, renders=len(sim_res.render_paths)
        )

    except Exception as e:
        logger.error("validation_node_error", error=str(e))
        state["simulation_result"] = {
            "valid": False,
            "cost": 0,
            "logs": [f"Validation error: {e!s}"],
            "render_paths": [],
        }

    return state


async def reviewer_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Agentic review of the generated benchmark.
    Uses vision to inspect renders and writes a persistent review file.
    """
    import base64
    from langgraph.prebuilt import create_react_agent
    from langchain_core.tools import tool

    logger.info("reviewer_node_start", round=state.get("review_round", 0))

    # Increment round
    state["review_round"] = state.get("review_round", 0) + 1
    current_round = state["review_round"]
    review_dir = Path(f"worker/reviews/review-round-{current_round}")
    review_dir.mkdir(parents=True, exist_ok=True)

    # Define restricted filesystem tools for the reviewer
    @tool
    def list_workspace(path: str = "."):
        """List files in the workspace. Use this to see what's available."""
        # Read-only access to most things
        try:
            p = Path(path)
            return [f.name for f in p.iterdir()]
        except Exception as e:
            return str(e)

    @tool
    def read_workspace_file(path: str):
        """Read a file from the workspace (read-only)."""
        try:
            return Path(path).read_text()
        except Exception as e:
            return str(e)

    @tool
    def write_review(content: str):
        """
        Write the final review file.
        MUST include YAML frontmatter with 'decision' (approved/rejected) and 'comments' (list).
        Save to the designated review folder for this round.
        """
        try:
            review_path = review_dir / "review.md"
            review_path.write_text(content)
            return f"Review saved to {review_path}"
        except Exception as e:
            return str(e)

    tools = [list_workspace, read_workspace_file, write_review]
    llm = ChatOpenAI(model="gpt-4o", temperature=0)

    # Prepare system prompt
    template_path = Path(__file__).parent / "templates" / "reviewer_prompt.txt"
    base_prompt = template_path.read_text()

    system_message = f"""{base_prompt}

You are an agentic reviewer. You have access to the filesystem to inspect code and renders.
You MUST:
1. Inspect the renders provided in the message.
2. Use the 'write_review' tool to persist your findings to '{review_dir}/review.md'.
3. The review file MUST start with a YAML frontmatter:
---
decision: approved  # or rejected
comments:
  - "Comment 1"
  - "Comment 2"
---

4. If you approve, you MUST include 'STATUS: APPROVE' in your final response.
5. If you reject, you MUST include 'STATUS: REJECT' and provide feedback.
"""

    # Load and encode images for vision model to be passed in the initial message
    renders = state.get("simulation_result", {}).get("render_paths", [])
    image_contents = []
    for path_str in renders[:8]:
        try:
            path = Path(path_str)
            if path.exists():
                with open(path, "rb") as f:
                    encoded = base64.b64encode(f.read()).decode("utf-8")
                    image_contents.append(
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/png;base64,{encoded}"},
                        }
                    )
        except Exception as e:
            logger.warning(
                "failed_to_load_render_for_vision", path=path_str, error=str(e)
            )

    user_content = [
        {
            "type": "text",
            "text": f"Please review the benchmark for theme: {state.get('plan', {}).get('theme', 'Unknown')}\nPrompt: {state['session'].prompt}",
        }
    ]
    user_content.extend(image_contents)

    # Create and run the react agent
    agent = create_react_agent(llm, tools, prompt=system_message)

    result = await agent.ainvoke({"messages": [HumanMessage(content=user_content)]})

    final_response = str(result["messages"][-1].content)

    # Extract status from the agent's reasoning/response
    if "APPROVE" in final_response.upper():
        state["review_feedback"] = "Approved"
    else:
        state["review_feedback"] = final_response

    logger.info("reviewer_node_complete", feedback=state["review_feedback"])
    return state
