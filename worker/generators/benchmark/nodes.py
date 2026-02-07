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
    Generates build123d script based on plan and feedback using deepagents.
    """
    from deepagents import create_deep_agent
    from worker.filesystem.backend import SandboxFilesystemBackend

    logger.info("coder_node_start", session_id=state["session"].session_id)

    # 1. Load template
    template_path = Path(__file__).parent / "templates" / "coder_prompt.txt"
    template = template_path.read_text()

    # 2. Format prompt context
    validation_logs = "\n".join(state["session"].validation_logs)
    if state.get("simulation_result") and not state["simulation_result"]["valid"]:
        validation_logs += "\n" + "\n".join(state["simulation_result"]["logs"])

    system_prompt = template.format(
        plan=json.dumps(state.get("plan"), indent=2),
        review_feedback=state.get("review_feedback", "No feedback provided."),
        validation_logs=validation_logs,
    )

    # 3. Setup Agent with deepagents
    llm = ChatOpenAI(model="gpt-4o", temperature=0)
    backend = SandboxFilesystemBackend.create(state["session"].session_id)
    
    agent = create_deep_agent(
        model=llm,
        backend=backend,
        system_prompt=system_prompt,
        name="CAD Coder Agent"
    )

    # 4. Invoke Agent
    result = await agent.ainvoke({"messages": [HumanMessage(content=f"Implement the benchmark script for: {state['session'].prompt}")]})
    
    # 5. Update state
    try:
        # read_raw to avoid line numbers
        s3_path = backend._resolve_path("script.py")
        if backend._fs.exists(s3_path):
            with backend._fs.open(s3_path, "rb") as f:
                state["current_script"] = f.read().decode("utf-8")
    except Exception as e:
        logger.warning("coder_agent_failed_to_read_script", error=str(e))

    state["messages"].extend(result["messages"])

    logger.info("coder_node_complete", script_length=len(state.get("current_script", "")))
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
    Agentic review of the generated benchmark using deepagents.
    Uses vision to inspect renders and verifies that only the review file is edited.
    """
    import base64
    import yaml
    from deepagents import create_deep_agent
    from deepagents.backends.protocol import BackendProtocol
    from worker.filesystem.backend import SandboxFilesystemBackend

    logger.info("reviewer_node_start", round=state.get("review_round", 0))

    # 1. Setup round and review path
    state["review_round"] = state.get("review_round", 0) + 1
    current_round = state["review_round"]
    review_filename = f"reviews/review-round-{current_round}/review.md"
    
    # 2. Vision: Load and encode images
    renders = state.get("simulation_result", {}).get("render_paths", [])
    image_contents = []
    for path_str in renders[:8]:
        try:
            path = Path(path_str)
            if path.exists():
                with open(path, "rb") as f:
                    encoded = base64.b64encode(f.read()).decode("utf-8")
                    image_contents.append({
                        "type": "image_url",
                        "image_url": {"url": f"data:image/png;base64,{encoded}"}
                    })
        except Exception as e:
            logger.warning("failed_to_load_render_for_vision", path=path_str, error=str(e))

    # 3. Backend Wrapper for validation (implementing BackendProtocol)
    class GuardedBackend(BackendProtocol):
        def __init__(self, inner, allowed_file: str):
            self.inner = inner
            self.allowed_file = allowed_file.lstrip("/")
            self.violations = []

        def __getattr__(self, name):
            return getattr(self.inner, name)

        def _check(self, file_path: str, op: str):
            p = file_path.lstrip("/")
            if p != self.allowed_file:
                self.violations.append(f"Attempted to {op} unauthorized path: {file_path}")

        def write(self, file_path, content):
            self._check(file_path, "write")
            return self.inner.write(file_path, content)

        async def awrite(self, file_path, content):
            self._check(file_path, "write")
            return await self.inner.awrite(file_path, content)

        def edit(self, file_path, old, new, replace_all=False):
            self._check(file_path, "edit")
            return self.inner.edit(file_path, old, new, replace_all)

        async def aedit(self, file_path, old, new, replace_all=False):
            self._check(file_path, "edit")
            return await self.inner.aedit(file_path, old, new, replace_all)

        def delete(self, file_path):
            self.violations.append(f"Attempted to delete path: {file_path}")
            return self.inner.delete(file_path)

        async def adelete(self, file_path):
            self.violations.append(f"Attempted to delete path: {file_path}")
            return await self.inner.adelete(file_path)

    base_backend = SandboxFilesystemBackend.create(state["session"].session_id)
    guarded_backend = GuardedBackend(base_backend, review_filename)

    # 4. Prepare system prompt
    template_path = Path(__file__).parent / "templates" / "reviewer_prompt.txt"
    base_prompt = template_path.read_text()
    
    system_prompt = f"""{base_prompt}

You are an agentic reviewer with access to the CAD workspace.
YOU MUST:
1. Inspect the renders provided in the vision message.
2. Inspect the generated code ('script.py') and MJCF.
3. Use the 'write_file' tool to persist your review ONLY to '{review_filename}'.
4. The review file MUST start with a YAML frontmatter:
---
decision: approved  # or rejected
comments:
  - "Comment 1"
  - "Comment 2"
---

YOUR ONLY ALLOWED WRITE OPERATION IS TO '{review_filename}'.
"""

    # 5. Setup Agent
    llm = ChatOpenAI(model="gpt-4o", temperature=0)
    agent = create_deep_agent(
        model=llm,
        backend=guarded_backend,
        system_prompt=system_prompt,
        name="Benchmark Reviewer Agent"
    )

    # 6. Invoke Agent with Vision
    user_content = [
        {"type": "text", "text": f"Please review the benchmark for theme: {state.get('plan', {}).get('theme', 'Unknown')}\nPrompt: {state['session'].prompt}"}
    ]
    user_content.extend(image_contents)

    result = await agent.ainvoke({"messages": [HumanMessage(content=user_content)]})
    
    # 7. Check violations and parse results
    if guarded_backend.violations:
        logger.error("reviewer_security_violations", violations=guarded_backend.violations)
        state["review_feedback"] = f"Reviewer security violation: {', '.join(guarded_backend.violations)}"
        return state

    # Parse review from backend
    try:
        s3_path = base_backend._resolve_path(review_filename)
        if base_backend._fs.exists(s3_path):
            with base_backend._fs.open(s3_path, "rb") as f:
                content = f.read().decode("utf-8")
                
            frontmatter_match = re.search(r"^---\s*\n(.*?)\n---\s*\n", content, re.DOTALL)
            if frontmatter_match:
                frontmatter = yaml.safe_load(frontmatter_match.group(1))
                if frontmatter.get("decision") == "approved":
                    state["review_feedback"] = "Approved"
                else:
                    state["review_feedback"] = "\n".join(frontmatter.get("comments", ["Rejected"]))
            else:
                state["review_feedback"] = "Error: Missing YAML frontmatter in review file."
        else:
            state["review_feedback"] = "Error: Review file not created by agent."
    except Exception as e:
        state["review_feedback"] = f"Error reading review file: {e!s}"

    logger.info("reviewer_node_complete", feedback=state["review_feedback"])
    return state
