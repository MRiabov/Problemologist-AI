import asyncio
import json
import os
import re
from pathlib import Path

import httpx
import structlog
import yaml
from langchain_core.messages import HumanMessage, SystemMessage
from langchain_core.tools import tool
from langchain_openai import ChatOpenAI
from langgraph.prebuilt import create_react_agent

from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.database import DatabaseCallbackHandler
from controller.observability.langfuse import get_langfuse_callback
from controller.prompts import get_prompt

from ..config import settings
from .state import BenchmarkGeneratorState
from .tools import get_benchmark_tools

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
    Breaks down the user prompt into a randomization strategy using a LangGraph node.
    """
    session_id = str(state["session"].session_id)
    worker_url = os.getenv("WORKER_URL", "http://worker:8001")

    async with httpx.AsyncClient() as http_client:
        client = WorkerClient(
            base_url=worker_url, session_id=session_id, http_client=http_client
        )

        logger.info("planner_node_start", session_id=session_id)

        # Load prompt from YAML config
        try:
            base_prompt = get_prompt("benchmark_generator.planner.system")
        except Exception as e:
            logger.error("planner_prompt_load_failed", error=str(e))
            state["plan"] = {"error": f"Failed to load planner prompt: {e}"}
            return state

        # Langfuse tracing
        langfuse_callback = get_langfuse_callback(
            name="benchmark_planner", session_id=session_id
        )
        # Database tracing for real-time tool tracking in UI
        db_callback = DatabaseCallbackHandler(episode_id=session_id)

        callbacks = [db_callback]
        if langfuse_callback:
            callbacks.append(langfuse_callback)

        llm = ChatOpenAI(model=settings.llm_model, temperature=0)

        # Initialize Git Repo on Worker
        await client.git_init()

        # Apply custom objectives if any
        custom_objectives = state["session"].custom_objectives
        if custom_objectives:
            try:
                # Read existing objectives.yaml
                obj_content = await client.read_file("objectives.yaml")
                obj_data = yaml.safe_load(obj_content)

                # Update constraints
                if "constraints" not in obj_data:
                    obj_data["constraints"] = {}

                if "max_unit_cost" in custom_objectives:
                    obj_data["constraints"]["max_unit_cost"] = custom_objectives[
                        "max_unit_cost"
                    ]
                if "max_weight" in custom_objectives:
                    obj_data["constraints"]["max_weight"] = custom_objectives[
                        "max_weight"
                    ]
                if "target_quantity" in custom_objectives:
                    obj_data["constraints"]["target_quantity"] = custom_objectives[
                        "target_quantity"
                    ]

                # Write back
                new_content = yaml.dump(obj_data, sort_keys=False)
                await client.write_file("objectives.yaml", new_content)
                logger.info(
                    "applied_custom_objectives",
                    session_id=session_id,
                    objectives=custom_objectives,
                )

            except Exception as e:
                logger.error(
                    "failed_to_apply_custom_objectives",
                    error=str(e),
                    session_id=session_id,
                )

        # Validation Gate for objectives.yaml if it was updated
        if custom_objectives:
            from worker.utils.file_validation import validate_node_output

            try:
                obj_content = await client.read_file("objectives.yaml")
                is_valid, errors = validate_node_output(
                    "planner", {"objectives.yaml": obj_content}
                )
                if not is_valid:
                    logger.warning("benchmark_planner_validation_failed", errors=errors)
                    # We don't fail the whole node yet, but we add to state
                    state["plan"]["validation_errors"] = errors
            except Exception:
                pass

        try:
            # We use the LLM directly as the planner doesn't currently need complex tools
            messages = [
                SystemMessage(content=base_prompt),
                # Include context from previous nodes (e.g. COTS search)
                *state.get("messages", []),
                HumanMessage(
                    content=f"User Request:\n{state['session'].prompt}\n\nPlease generate the randomization strategy JSON now."
                ),
            ]
            response = await llm.ainvoke(messages, config={"callbacks": callbacks})
        except Exception as e:
            logger.error(
                "planner_agent_run_failed", error=str(e), session_id=session_id
            )
            raise RuntimeError(f"Planner agent failed: {e}") from e

        try:
            content = str(response.content)
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

        state["messages"].append(response)

        logger.info("planner_node_complete", plan=state["plan"])
    return state


async def coder_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Generates build123d script and validates it.
    """

    session_id = str(state["session"].session_id)
    worker_url = os.getenv("WORKER_URL", "http://worker:8001")

    async with httpx.AsyncClient() as http_client:
        client = WorkerClient(
            base_url=worker_url, session_id=session_id, http_client=http_client
        )

        logger.info("coder_node_start", session_id=session_id)

        # 1. Load prompt from YAML
        try:
            base_prompt = get_prompt("benchmark_generator.coder.system")
        except Exception as e:
            logger.error("coder_prompt_load_failed", error=str(e))
            base_prompt = "Error loading prompt."

        # 2. Format prompt context
        validation_logs = "\n".join(state["session"].validation_logs)
        if state.get("simulation_result") and not state["simulation_result"]["valid"]:
            validation_logs += "\n" + "\n".join(state["simulation_result"]["logs"])

        # Read objectives.yaml from worker if it exists
        objectives_yaml = "# No objectives.yaml found."
        try:
            files = await client.list_files(".")
            if any(f.path.endswith("objectives.yaml") for f in files):
                resp = await client.read_file("objectives.yaml")
                objectives_yaml = resp
        except Exception as e:
            logger.warning(f"failed_to_read_objectives_yaml: {e}")

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

        # 3. Setup ReAct Agent
        middleware = RemoteFilesystemMiddleware(client)
        tools = get_benchmark_tools(middleware)

        # Langfuse tracing
        langfuse_callback = get_langfuse_callback(
            name="benchmark_coder", session_id=session_id
        )
        # Database tracing for real-time tool tracking in UI
        db_callback = DatabaseCallbackHandler(episode_id=session_id)

        callbacks = [db_callback]
        if langfuse_callback:
            callbacks.append(langfuse_callback)

        llm = ChatOpenAI(model=settings.llm_model, temperature=0)

        # 4. Invoke Agent using manual tool loop to avoid nested graphs
        llm_with_tools = llm.bind_tools(tools)
        messages = (
            [SystemMessage(content=system_prompt)]
            + state.get("messages", [])
            + [
                HumanMessage(
                    content=f"Implement the benchmark script for: {state['session'].prompt}"
                )
            ]
        )

        # Basic ReAct loop
        for _ in range(5):
            response = await llm_with_tools.ainvoke(
                messages, config={"callbacks": callbacks}
            )
            messages.append(response)

            if not response.tool_calls:
                break

            for tool_call in response.tool_calls:
                tool_name = tool_call["name"]
                tool_args = tool_call["args"]
                # Find matching tool
                tool_func = next((t for t in tools if t.name == tool_name), None)
                if tool_func:
                    observation = await tool_func.ainvoke(tool_args)
                    messages.append(
                        {
                            "role": "tool",
                            "tool_call_id": tool_call["id"],
                            "content": str(observation),
                        }
                    )
                else:
                    messages.append(
                        {
                            "role": "tool",
                            "tool_call_id": tool_call["id"],
                            "content": f"Error: Tool '{tool_name}' not found.",
                        }
                    )

        # 5. Update state
        try:
            state["current_script"] = await client.read_file("script.py")
        except Exception as e:
            logger.warning("coder_agent_failed_to_read_script", error=str(e))

        state["messages"] = messages

        # T015: Validation Gate for benchmark script
        if state.get("current_script"):
            from worker.utils.file_validation import validate_node_output

            is_valid, errors = validate_node_output(
                "coder", {"script.py": state["current_script"]}
            )
            if not is_valid:
                logger.warning("benchmark_coder_validation_failed", errors=errors)
                # Add validation errors to logs for the next loop
                state["session"].validation_logs.append(
                    f"Output validation failed: {errors}"
                )

        logger.info(
            "coder_node_complete", script_length=len(state.get("current_script", ""))
        )

        # Now run validation (merged from validator_node)
        script = state.get("current_script")
        if script:
            logger.info("running_integrated_validation", session_id=session_id)
            try:
                # geometric validation
                val_res = await client.validate(script_path="script.py")
                if not val_res.success:
                    state["simulation_result"] = {
                        "valid": False,
                        "cost": 0,
                        "logs": [f"Geometric validation failed: {val_res.message}"],
                        "render_paths": [],
                        "render_data": [],
                    }
                else:
                    # physics simulation
                    sim_res = await client.simulate(script_path="script.py")
                    if not sim_res.success:
                        state["simulation_result"] = {
                            "valid": False,
                            "cost": 0,
                            "logs": [f"Physics simulation failed: {sim_res.message}"],
                            "render_paths": [],
                            "render_data": [],
                        }
                    else:
                        # Download Renders
                        render_paths = (
                            sim_res.artifacts.get("render_paths", [])
                            if sim_res.artifacts
                            else []
                        )

                        async def _download_render(hc: httpx.AsyncClient, p: str):
                            url = f"{worker_url}/assets/{p.lstrip('/')}"
                            try:
                                resp = await hc.get(
                                    url, headers={"X-Session-ID": session_id}
                                )
                                if resp.status_code == 200:
                                    return resp.content
                            except Exception:
                                pass
                            return None

                        tasks = [_download_render(http_client, p) for p in render_paths]
                        results = await asyncio.gather(*tasks)
                        render_data = [r for r in results if r is not None]

                        state["simulation_result"] = {
                            "valid": True,
                            "cost": 0,
                            "logs": ["Validation passed."],
                            "render_paths": render_paths,
                            "render_data": render_data,
                        }
            except Exception as e:
                logger.error("integrated_validation_error", error=str(e))

    return state


async def cots_search_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    COTS Search node for benchmark generation.
    """
    session_id = str(state["session"].session_id)
    worker_url = os.getenv("WORKER_URL", "http://worker:8001")

    async with httpx.AsyncClient() as http_client:
        client = WorkerClient(
            base_url=worker_url, session_id=session_id, http_client=http_client
        )

        from shared.cots.agent import search_cots_catalog

        llm = ChatOpenAI(model=settings.llm_model, temperature=0)
        tools = [search_cots_catalog]
        agent = create_react_agent(llm, tools)

        # Langfuse tracing
        langfuse_callback = get_langfuse_callback(
            name="benchmark_cots_search", session_id=session_id
        )
        db_callback = DatabaseCallbackHandler(episode_id=session_id)

        callbacks = [db_callback]
        if langfuse_callback:
            callbacks.append(langfuse_callback)

        prompt = f"Find components for the benchmark: {state['session'].prompt}"
        result = await agent.ainvoke(
            {"messages": [HumanMessage(content=prompt)]},
            config={"callbacks": callbacks},
        )

        state["messages"].extend(result["messages"])
    return state


async def skills_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Skills node for benchmark generation: Identifies patterns and suggests skills.
    """
    session_id = str(state["session"].session_id)
    # Langfuse tracing
    langfuse_callback = get_langfuse_callback(
        name="benchmark_skills", session_id=session_id
    )
    db_callback = DatabaseCallbackHandler(episode_id=session_id)

    callbacks = [db_callback]
    if langfuse_callback:
        callbacks.append(langfuse_callback)

    logger.info(
        "skills_node_start",
        session_id=session_id,
        db_enabled=db_callback is not None,
        tracing_enabled=langfuse_callback is not None,
    )

    try:
        base_prompt = get_prompt("subagents.skill_learner.system")
    except Exception as e:
        logger.error("skills_prompt_load_failed", error=str(e))
        return state

    # Construct context from session
    # We don't have a linear journal, so we construct one from messages and results
    journal_content = f"Task: {state['session'].prompt}\n\nExecution Logs:\n"

    # Add validation logs
    if state["session"].validation_logs:
        journal_content += "Validation Logs:\n" + "\n".join(state["session"].validation_logs) + "\n"

    # Add simulation result summary
    if state.get("simulation_result"):
        sim = state["simulation_result"]
        journal_content += f"\nSimulation Result: {'Valid' if sim.get('valid') else 'Invalid'}\n"
        if sim.get("logs"):
            journal_content += "Simulation Logs:\n" + "\n".join(sim["logs"]) + "\n"

    # Add tool outputs from messages
    journal_content += "\nTool Interactions:\n"
    for msg in state.get("messages", []):
        if hasattr(msg, "content") and msg.content:
            # Simplify content to avoid huge logs
            content_preview = str(msg.content)[:500]
            journal_content += f"- {msg.type}: {content_preview}...\n"

    llm = ChatOpenAI(model=settings.llm_model, temperature=0)

    messages = [
        SystemMessage(content=base_prompt),
        HumanMessage(content=f"Please analyze this execution journal:\n{journal_content}")
    ]

    try:
        response = await llm.ainvoke(messages, config={"callbacks": callbacks})
        content = str(response.content)

        # Parse for skills
        if "SKILL:" in content and "CONTENT:" in content:
            parts = content.split("CONTENT:")
            raw_title = parts[0].replace("SKILL:", "").strip().lower()
            # Sanitize title to prevent path traversal
            title = re.sub(r"[^a-z0-9_]", "_", raw_title)
            title = re.sub(r"_+", "_", title).strip("_")

            if not title:
                title = "untitled_skill"

            skill_content = parts[1].strip()

            # Save locally to suggested_skills
            skills_dir = Path("suggested_skills")
            skills_dir.mkdir(exist_ok=True)

            file_path = skills_dir / f"{title}.md"
            with file_path.open("w") as f:
                f.write(skill_content)

            logger.info("suggested_new_skill", title=title, path=str(file_path))
        else:
            logger.info("no_new_skills_identified")

    except Exception as e:
        logger.error("skills_node_failed", error=str(e))

    return state


async def reviewer_node(state: BenchmarkGeneratorState) -> BenchmarkGeneratorState:
    """
    Agentic review of the generated benchmark using a LangGraph node.
    Uses vision to inspect renders and verifies that only the review file is edited.
    """
    import base64

    session_id = str(state["session"].session_id)
    worker_url = os.getenv("WORKER_URL", "http://worker:8001")

    async with httpx.AsyncClient() as http_client:
        client = WorkerClient(
            base_url=worker_url, session_id=session_id, http_client=http_client
        )

        logger.info("reviewer_node_start", round=state.get("review_round", 0))

        # 1. Setup round and review path
        state["review_round"] = state.get("review_round", 0) + 1
        current_round = state["review_round"]
        review_filename = f"reviews/review-round-{current_round}/review.md"

        # 2. Vision: Load and encode images
        render_data = state.get("simulation_result", {}).get("render_data", [])
        image_contents = []

        for img_bytes in (render_data or [])[:8]:
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

        # 3. Setup ReAct Agent with guarded tools
        middleware = RemoteFilesystemMiddleware(client)
        all_tools = get_benchmark_tools(middleware)

        # Filter tools to only allow write_file to the specific review path
        violations = []

        @tool
        async def write_review_file(path: str, content: str) -> str:
            """Write the review to the review file."""
            p = path.lstrip("/")
            if p != review_filename.lstrip("/"):
                violations.append(f"Attempted to write unauthorized path: {path}")
                return "Error: Unauthorized path."
            success = await middleware.write_file(path, content)
            return (
                "Review written successfully." if success else "Error writing review."
            )

        # Keep only safe tools
        safe_tools = [
            t
            for t in all_tools
            if t.name not in ("write_file", "edit_file", "submit_for_review")
        ]
        safe_tools.append(write_review_file)

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
2. Inspect the generated code ('script.py').
3. Use the 'write_review_file' tool to persist your review ONLY to '{review_filename}'.
4. The review file MUST start with a YAML frontmatter:
---
decision: approved  # or rejected
comments:
  - "Brief issue description (max 100 chars)"
---

YOUR ONLY ALLOWED WRITE OPERATION IS TO '{review_filename}'.
"""

        # 5. Setup Agent
        langfuse_callback = get_langfuse_callback(
            name="benchmark_reviewer", session_id=session_id
        )
        db_callback = DatabaseCallbackHandler(episode_id=session_id)

        callbacks = [db_callback]
        if langfuse_callback:
            callbacks.append(langfuse_callback)

        llm = ChatOpenAI(model=settings.llm_model, temperature=0)

        # 6. Invoke Agent with Vision using manual loop
        llm_with_tools = llm.bind_tools(safe_tools)
        user_content = [
            {
                "type": "text",
                "text": f"Please review the benchmark for theme: {state.get('plan', {}).get('theme', 'Unknown')}\nPrompt: {state['session'].prompt}",
            }
        ]
        user_content.extend(image_contents)

        messages = [
            SystemMessage(content=system_prompt),
            HumanMessage(content=user_content),
        ]

        for _ in range(3):
            response = await llm_with_tools.ainvoke(
                messages, config={"callbacks": callbacks}
            )
            messages.append(response)

            if not response.tool_calls:
                break

            for tool_call in response.tool_calls:
                tool_name = tool_call["name"]
                tool_args = tool_call["args"]
                tool_func = next((t for t in safe_tools if t.name == tool_name), None)
                if tool_func:
                    observation = await tool_func.ainvoke(tool_args)
                    messages.append(
                        {
                            "role": "tool",
                            "tool_call_id": tool_call["id"],
                            "content": str(observation),
                        }
                    )
                else:
                    messages.append(
                        {
                            "role": "tool",
                            "tool_call_id": tool_call["id"],
                            "content": f"Error: Tool '{tool_name}' not found.",
                        }
                    )

        result_messages = messages

        # 7. Check violations and parse results
        if violations:
            logger.error("reviewer_security_violations", violations=violations)
            state["review_feedback"] = (
                f"Reviewer security violation: {', '.join(violations)}"
            )
            return state

        # Parse review from worker
        try:
            parent_dir = str(Path(review_filename).parent)
            files = await client.list_files(parent_dir)
            basename = Path(review_filename).name
            exists = any(f.path.endswith(basename) for f in files)

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
                        else:
                            state["review_feedback"] = "\n".join(
                                frontmatter.comments or ["Rejected"]
                            )
                    except ValidationError as e:
                        state["review_feedback"] = f"Invalid review frontmatter: {e}"
                else:
                    state["review_feedback"] = (
                        "Error: Missing YAML frontmatter in review file."
                    )
            else:
                state["review_feedback"] = "Error: Review file not created by agent."
        except Exception as e:
            state["review_feedback"] = f"Error reading review file: {e!s}"

        state["messages"] = result_messages
        logger.info("reviewer_node_complete", feedback=state["review_feedback"])
    return state
