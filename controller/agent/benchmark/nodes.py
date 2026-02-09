import json
import os
import re


import structlog
from deepagents import create_deep_agent
from langchain_core.messages import HumanMessage
from langchain_openai import ChatOpenAI

from controller.clients.backend import RemoteFilesystemBackend
from controller.clients.worker import WorkerClient
from controller.prompts import get_prompt

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
    Breaks down the user prompt into a concrete randomization strategy using a Deep Agent.
    """
    session_id = str(state["session"].session_id)
    worker_url = os.getenv("WORKER_URL", "http://worker:8001")
    client = WorkerClient(base_url=worker_url, session_id=session_id)

    logger.info("planner_node_start", session_id=session_id)

    # Load prompt from YAML config
    try:
        base_prompt = get_prompt("benchmark_generator.planner.system")
    except Exception as e:
        logger.error("planner_prompt_load_failed", error=str(e))
        state["plan"] = {"error": f"Failed to load planner prompt: {e}"}
        return state

    # The prompt is now part of the system instructions for the Deep Agent
    # We append the user prompt as context
    system_prompt = f"{base_prompt}\n\nUser Request:\n{state['session'].prompt}"

    llm = ChatOpenAI(model="gpt-4o", temperature=0)

    # Use a remote backend
    backend = RemoteFilesystemBackend(client)

    # Initialize Git Repo on Worker
    await client.git_init()

    agent = create_deep_agent(
        model=llm,
        backend=backend,
        system_prompt=system_prompt,
        name="Benchmark Planner Agent",
    )

    # We send a trigger message to start the agent's thought process
    # The actual context is in the system prompt
    response = await agent.ainvoke(
        {
            "messages": [
                HumanMessage(
                    content="Please generate the randomization strategy JSON now."
                )
            ]
        }
    )

    try:
        # Extract JSON from the agent's final answer
        # DeepAgent's result["messages"][-1].content should contain the answer
        content = response["messages"][-1].content
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

    # Store messages for debugging/trace
    state["messages"].extend(response["messages"])

    logger.info("planner_node_complete", plan=state["plan"])
    return state


async def coder_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Generates build123d script based on plan and feedback using deepagents.
    """
    session_id = str(state["session"].session_id)
    worker_url = os.getenv("WORKER_URL", "http://worker:8001")
    client = WorkerClient(base_url=worker_url, session_id=session_id)

    logger.info("coder_node_start", session_id=session_id)

    # 1. Load prompt from YAML
    try:
        base_prompt = get_prompt("benchmark_generator.coder.system")
    except Exception as e:
        logger.error("coder_prompt_load_failed", error=str(e))
        # If we can't load the prompt, we can't really proceed effectively.
        # But we might want to return state to show error in UI?
        # For now let's just log and continue with empty string (will fail later likely)
        base_prompt = "Error loading prompt."

    # 2. Format prompt context
    validation_logs = "\n".join(state["session"].validation_logs)
    if state.get("simulation_result") and not state["simulation_result"]["valid"]:
        validation_logs += "\n" + "\n".join(state["simulation_result"]["logs"])

    # Read objectives.yaml from worker if it exists
    objectives_yaml = "# No objectives.yaml found."
    try:
        # We use the client directly to check if file exists and read it
        files = await client.list_files(".")
        if any(f.path.endswith("objectives.yaml") for f in files):
            resp = await client.read_file("objectives.yaml")
            objectives_yaml = resp
    except Exception as e:
        raise ValueError(f"failed_to_read_objectives_yaml: {e}")

    # Appending context to the base system prompt
    system_prompt = f"""{base_prompt}

### Context
Original User Request:
{state["session"].prompt}

Plan:
{json.dumps(state.get("plan"), indent=2)}

### Draft Objectives (YAML):
{objectives_yaml}

Review Feedback:
{state.get("review_feedback", "No feedback provided.")}

Validation Logs:
{validation_logs}
"""

    # 3. Setup Agent with deepagents
    llm = ChatOpenAI(model="gpt-4o", temperature=0)
    backend = RemoteFilesystemBackend(client)

    agent = create_deep_agent(
        model=llm, backend=backend, system_prompt=system_prompt, name="CAD Coder Agent"
    )

    # 4. Invoke Agent
    result = await agent.ainvoke(
        {
            "messages": [
                HumanMessage(
                    content=f"Implement the benchmark script for: {state['session'].prompt}"
                )
            ]
        }
    )

    # 5. Update state
    try:
        # read raw content from worker
        state["current_script"] = await client.read_file("script.py")
    except Exception as e:
        logger.warning("coder_agent_failed_to_read_script", error=str(e))

    state["messages"].extend(result["messages"])

    logger.info(
        "coder_node_complete", script_length=len(state.get("current_script", ""))
    )
    return state


async def validator_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Validates the generated script using physics simulation and geometric checks via Worker API.
    """
    import httpx

    script = state.get("current_script")
    if not script:
        return state

    session_id = str(state["session"].session_id)
    worker_url = os.getenv("WORKER_URL", "http://worker:8001")
    client = WorkerClient(base_url=worker_url, session_id=session_id)

    logger.info("validator_node_start", session_id=session_id)

    try:
        # 0. Git Integration: Commit on Worker
        try:
            commit_res = await client.git_commit(
                f"Checkpoint before validation. Script length: {len(script)}"
            )
            if commit_res.success and commit_res.commit_hash:
                logger.info("git_commit_success", commit_hash=commit_res.commit_hash)
        except Exception as e:
            logger.warning("git_commit_failed", error=str(e))

        # 1. Geometric Validation
        val_res = await client.validate(script_path="script.py")
        if not val_res.success:
            state["simulation_result"] = {
                "valid": False,
                "cost": 0,
                "logs": [f"Geometric validation failed: {val_res.message}"],
                "render_paths": [],
                "render_data": [],
            }
            return state

        # 2. Physics Simulation
        sim_res = await client.simulate(script_path="script.py")
        if not sim_res.success:
            state["simulation_result"] = {
                "valid": False,
                "cost": 0,
                "logs": [f"Physics simulation failed: {sim_res.message}"],
                "render_paths": [],
                "render_data": [],
            }
            return state

        # 3. Download Renders
        # Worker returns paths like "/tmp/.../render_0.png"? No, routes says:
        # result.render_paths. Ideally these are relative or we can fetch them via /assets/
        # api_simulate returns result.render_paths.
        render_data = []
        render_paths = (
            sim_res.artifacts.get("render_paths", []) if sim_res.artifacts else []
        )

        for path in render_paths:
            async with httpx.AsyncClient() as http_client:
                url = f"{worker_url}/assets/{path.lstrip('/')}"
                try:
                    resp = await http_client.get(
                        url, headers={"X-Session-ID": session_id}
                    )
                    if resp.status_code == 200:
                        render_data.append(resp.content)
                    else:
                        logger.warning(
                            "failed_to_download_render",
                            path=path,
                            status=resp.status_code,
                        )
                except Exception as e:
                    logger.warning(
                        "failed_to_download_render_exception", path=path, error=str(e)
                    )

        state["simulation_result"] = {
            "valid": True,
            "cost": 0,
            "logs": ["Validation passed."],
            "render_paths": render_paths,
            "render_data": render_data,
        }
        logger.info("validator_node_complete", success=True, renders=len(render_data))

    except Exception as e:
        logger.error("validator_node_remote_error", error=str(e))
        state["simulation_result"] = {
            "valid": False,
            "cost": 0,
            "logs": [f"Remote validation error: {e!s}"],
            "render_paths": [],
            "render_data": [],
        }

    return state


async def reviewer_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Agentic review of the generated benchmark using deepagents.
    Uses vision to inspect renders and verifies that only the review file is edited.
    """
    import base64

    import yaml
    from deepagents.backends.protocol import BackendProtocol

    session_id = str(state["session"].session_id)
    worker_url = os.getenv("WORKER_URL", "http://worker:8001")
    client = WorkerClient(base_url=worker_url, session_id=session_id)

    logger.info("reviewer_node_start", round=state.get("review_round", 0))

    # 1. Setup round and review path
    state["review_round"] = state.get("review_round", 0) + 1
    current_round = state["review_round"]
    review_filename = f"reviews/review-round-{current_round}/review.md"

    # 2. Vision: Load and encode images
    render_data = state.get("simulation_result", {}).get("render_data", [])
    image_contents = []

    # We expect render_data to be list[bytes]
    for img_bytes in render_data[:8]:
        try:
            if img_bytes:
                encoded = base64.b64encode(img_bytes).decode("utf-8")
                image_contents.append(
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/png;base64,{encoded}"},
                    }
                )
        except Exception as e:
            logger.warning("failed_to_encode_render_for_vision", error=str(e))

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
                self.violations.append(
                    f"Attempted to {op} unauthorized path: {file_path}"
                )

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

    base_backend = RemoteFilesystemBackend(client)
    guarded_backend = GuardedBackend(base_backend, review_filename)

    # 4. Prepare system prompt
    try:
        base_prompt = get_prompt("benchmark_generator.reviewer.system")
    except Exception as e:
        logger.error("reviewer_prompt_load_failed", error=str(e))
        base_prompt = "You are an agentic reviewer."

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
        name="Benchmark Reviewer Agent",
    )

    # 6. Invoke Agent with Vision
    user_content = [
        {
            "type": "text",
            "text": f"Please review the benchmark for theme: {state.get('plan', {}).get('theme', 'Unknown')}\nPrompt: {state['session'].prompt}",
        }
    ]
    user_content.extend(image_contents)

    await agent.ainvoke({"messages": [HumanMessage(content=user_content)]})

    # 7. Check violations and parse results
    if guarded_backend.violations:
        logger.error(
            "reviewer_security_violations", violations=guarded_backend.violations
        )
        state["review_feedback"] = (
            f"Reviewer security violation: {', '.join(guarded_backend.violations)}"
        )
        return state

    # Parse review from backend
    try:
        # Check if review file exists
        files = await client.list_files(os.path.dirname(review_filename))
        exists = any(f.path.endswith(os.path.basename(review_filename)) for f in files)

        if exists:
            content = await client.read_file(review_filename)

            frontmatter_match = re.search(
                r"^---\s*\n(.*?)\n---\s*\n", content, re.DOTALL
            )
            if frontmatter_match:
                from pydantic import ValidationError

                from shared.models.schemas import ReviewFrontmatter

                try:
                    raw_frontmatter = yaml.safe_load(frontmatter_match.group(1))
                    frontmatter = ReviewFrontmatter(**raw_frontmatter)

                    if frontmatter.decision == "approved":
                        state["review_feedback"] = "Approved"
                    elif frontmatter.decision in (
                        "confirm_plan_refusal",
                        "reject_plan_refusal",
                    ):
                        state["review_feedback"] = (
                            f"Plan refusal {frontmatter.decision}: "
                            + "\n".join(frontmatter.comments)
                        )
                    else:
                        state["review_feedback"] = "\n".join(
                            frontmatter.comments or ["Rejected"]
                        )
                except ValidationError as e:
                    errors = [f"{err['loc']}: {err['msg']}" for err in e.errors()]
                    state["review_feedback"] = (
                        f"Invalid review frontmatter: {'; '.join(errors)}"
                    )
            else:
                state["review_feedback"] = (
                    "Error: Missing YAML frontmatter in review file."
                )
        else:
            state["review_feedback"] = "Error: Review file not created by agent."
    except Exception as e:
        state["review_feedback"] = f"Error reading review file: {e!s}"

    logger.info("reviewer_node_complete", feedback=state["review_feedback"])
    return state
