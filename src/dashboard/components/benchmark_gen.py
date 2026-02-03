import asyncio
import re
import shutil
import tempfile
import traceback
from pathlib import Path

import streamlit as st
from langchain_core.messages import HumanMessage

from src.agent.graph.nodes.planner import planner_node
from src.generators.benchmark.agent import DEFAULT_RUNTIME_CONFIG, generator_agent
from src.generators.benchmark.manager import execute_build
from src.generators.benchmark.renderer import render_scenario
from src.generators.benchmark.validator import validate_mjcf


def init_bg_state():
    """Initializes or resets the benchmark generator state."""
    return {
        "stage": "INPUT",  # INPUT, PLANNING, PLAN_APPROVAL, CODING, CAD_APPROVAL, FINAL
        "request": "",
        "plan": "",
        "planner_reasoning": "",
        "code": "",
        "coder_reasoning": "",
        "mjcf": "",
        "errors": None,
        "attempts": 0,
        "attempt_history": [],  # List of {attempt, reasoning, code, errors}
        "renders": [],
    }


def render_benchmark_generator():
    st.header("Interactive Benchmark Generator")

    # Initialize state
    if "bg_state" not in st.session_state:
        st.session_state.bg_state = init_bg_state()

    state = st.session_state.bg_state

    if state["stage"] == "INPUT":
        render_input_stage(state)
    elif state["stage"] == "PLANNING":
        render_planning_stage(state)
    elif state["stage"] == "PLAN_APPROVAL":
        render_plan_approval_stage(state)
    elif state["stage"] == "CODING":
        render_coding_stage(state)
    elif state["stage"] == "CAD_APPROVAL":
        render_cad_approval_stage(state)
    elif state["stage"] == "FINAL":
        render_final_stage(state)

    if st.button("Reset Generator"):
        st.session_state.bg_state = init_bg_state()
        st.rerun()


def render_input_stage(state):
    st.subheader("1. Describe the Benchmark")
    request = st.text_area(
        "What should this benchmark test or teach?",
        placeholder="e.g., A puzzle where a robot must unscrew a cap from a bottle...",
    )
    if st.button("Generate Plan"):
        if request:
            state["request"] = request
            state["stage"] = "PLANNING"
            st.rerun()
        else:
            st.error("Please enter a description.")

    st.divider()
    with st.expander("🛠️ Debug: Insert Own Plan"):
        plan_input = st.text_area("Implementation Plan", height=150)
        if st.button("Insert Plan & Skip"):
            if plan_input:
                state["request"] = "Manual Plan Insertion"
                state["plan"] = plan_input
                state["planner_reasoning"] = "Manually inserted via debug tool."
                state["stage"] = "PLAN_APPROVAL"
                st.success("Plan inserted! Skipping to approval...")
                st.rerun()
            else:
                st.error("Please enter a plan.")


def render_planning_stage(state):
    with st.spinner("Agent is generating plan..."):
        # We simulate the planner node call
        try:
            input_state = {
                "messages": [HumanMessage(content=state["request"])],
                "step_count": 0,
                "runtime_config": DEFAULT_RUNTIME_CONFIG,
            }
            result = planner_node(input_state)
            state["plan"] = result["plan"]
            state["planner_reasoning"] = "Plan generated via DeepAgents planner."
            state["stage"] = "PLAN_APPROVAL"
            st.rerun()
        except Exception as e:
            st.error(f"Planning failed: {e}")
            st.code(traceback.format_exc())
            state["stage"] = "INPUT"


def render_plan_approval_stage(state):
    st.subheader("2. Review & Edit Plan")

    if state["planner_reasoning"]:
        with st.expander("🧠 View Agent Reasoning", expanded=False):
            st.markdown(state["planner_reasoning"])

    st.info(
        "The plan includes test objectives, rough geometry, and self-collision checks."
    )

    edited_plan = st.text_area("Edit Plan", value=state["plan"], height=300)

    col1, col2 = st.columns(2)
    with col1:
        if st.button("Approve Plan"):
            state["plan"] = edited_plan
            state["stage"] = "CODING"
            st.rerun()
    with col2:
        if st.button("Reject & Restart"):
            state["stage"] = "INPUT"
            st.rerun()


def render_coding_stage(state):
    st.subheader("3. Generating CAD Model")

    # We use a status container for detailed progress
    with st.status("DeepAgents Graph is active...", expanded=True) as status:
        # Clear history for new run if just started
        if state["attempts"] == 0:
            state["attempt_history"] = []

        try:
            input_state = {
                "messages": [HumanMessage(content=state["request"])],
                "plan": state["plan"],
                "step_count": 0,
                "runtime_config": DEFAULT_RUNTIME_CONFIG,
            }

            # We'll collect the final state here
            final_state = None

            async def run_and_stream():
                nonlocal final_state
                # We use 'updates' mode to see node transitions
                async for event in generator_agent.astream(
                    input_state, config={"recursion_limit": 200}, stream_mode="updates"
                ):
                    for node_name, updates in event.items():
                        if node_name == "planner":
                            status.update(
                                label="Step: Refining implementation plan...",
                                state="running",
                            )
                        elif node_name == "actor":
                            status.write(
                                "🤖 **Agent** is generating code or calling tools..."
                            )
                            status.update(
                                label="Step: Agent implementation...", state="running"
                            )
                        elif node_name == "tools":
                            if "messages" in updates:
                                for msg in updates["messages"]:
                                    if hasattr(msg, "name"):
                                        status.write(f"🛠️ Tool `{msg.name}` executed.")
                                        if msg.name == "validate_benchmark_model":
                                            if "Validation Passed!" in msg.content:
                                                status.write(
                                                    "✅ Validation **passed**!"
                                                )
                                            else:
                                                status.write(
                                                    "❌ Validation **failed**."
                                                )
                        elif node_name == "critic":
                            status.update(
                                label="Step: Critic reviewing work...", state="running"
                            )
                            status.write("🧐 **Critic** is checking the result...")
                        elif node_name == "skill_populator":
                            status.update(
                                label="Step: Recording lessons learned...",
                                state="running",
                            )
                            status.write("📝 Updating specialized CAD skills...")

                    # Merge updates into a local state to keep track of the latest
                    if final_state is None:
                        final_state = input_state.copy()

                    # Update messages (append)
                    if "messages" in updates:
                        final_state["messages"].extend(updates["messages"])
                    # Update other fields (overwrite)
                    for k, v in updates.items():
                        if k != "messages":
                            final_state[k] = v

            # Run the async stream
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(run_and_stream())

            # If final_state is still None, something went wrong
            if final_state is None:
                raise ValueError("Graph execution did not return any state.")

            # Analyze results from final_state
            messages = final_state["messages"]

            last_mjcf = ""
            last_code = ""

            # Extraction using markers
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

            # Extract code from the last validation call
            for msg in reversed(messages):
                if hasattr(msg, "tool_calls"):
                    for tc in msg.tool_calls:
                        if tc["name"] == "validate_benchmark_model":
                            last_code = tc["args"].get("code", "")
                            break
                if last_code:
                    break

            # Fallback for code: check write_script
            if not last_code:
                for msg in reversed(messages):
                    if hasattr(msg, "tool_calls"):
                        for tc in msg.tool_calls:
                            if tc["name"] == "write_script":
                                last_code = tc["args"].get("content", "")
                                break
                    if last_code:
                        break

            state["code"] = last_code
            state["mjcf"] = last_mjcf
            state["attempts"] = final_state.get("step_count", 0)

            if last_mjcf:
                state["errors"] = None
                status.update(label="Generation Successful!", state="complete")

                with tempfile.TemporaryDirectory() as tmpdir:
                    prefix = Path(tmpdir) / "preview"
                    state["renders"] = render_scenario(state["mjcf"], str(prefix))
                    persist_renders(state)

                state["stage"] = "CAD_APPROVAL"
                st.rerun()
            else:
                state["errors"] = (
                    "Generation failed to produce valid MJCF within limits."
                )
                status.update(label="Generation Failed", state="error")
                state["stage"] = "CAD_APPROVAL"
                st.rerun()

        except Exception as e:
            st.error(f"Coding stage failed: {e}")
            st.code(traceback.format_exc())
            state["stage"] = "PLAN_APPROVAL"


def persist_renders(state):
    render_dir = Path(".agent_storage") / "dashboard_renders"
    render_dir.mkdir(parents=True, exist_ok=True)
    new_paths = []
    for p in state["renders"]:
        basename = Path(p).name
        dest = render_dir / basename
        shutil.copy(str(p), str(dest))
        new_paths.append(str(dest))
    state["renders"] = new_paths


def render_cad_approval_stage(state):
    st.subheader("4. Review CAD Model & Debug Trace")

    if state["errors"]:
        st.error(
            f"Validation failed after {state['attempts']} attempts. "
            f"Last error: {state['errors']}"
        )
    else:
        st.success(f"Validation passed after {state['attempts']} attempts!")

    # 1. Debug History
    with st.expander("🧠 Debug: Agent Trace & History", expanded=True):
        st.write("### Planner Reasoning")
        st.markdown(state["planner_reasoning"] or "No reasoning provided.")
        st.divider()

        st.write("### Coder Attempt History")
        for attempt in state["attempt_history"]:
            st.write(f"#### Attempt {attempt['attempt']}")
            col_a, col_b = st.columns([1, 1])
            with col_a:
                st.write("**Reasoning:**")
                st.markdown(attempt["reasoning"] or "No reasoning.")
                if attempt["errors"]:
                    st.error(f"**Error:**\n{attempt['errors']}")
                else:
                    st.success("**Status:** Passed Validation")
            with col_b:
                st.write("**Generated Code:**")
                st.code(attempt["code"], language="python")
            st.divider()

    col1, col2 = st.columns([1, 1])

    with col1:
        st.write("### Current Python Code")
        st.code(state["code"], language="python")

        edited_code = st.text_area(
            "Manual Edits (optional)", value=state["code"], height=300
        )

        if st.button("Re-validate with Edits"):
            state["code"] = edited_code
            try:
                mjcf_xml = execute_build(state["code"], seed=0)
                report = validate_mjcf(mjcf_xml)

                if report["is_valid"]:
                    state["mjcf"] = mjcf_xml
                    state["errors"] = None
                    # Add manual edit to history
                    state["attempt_history"].append(
                        {
                            "attempt": "Manual Edit",
                            "reasoning": "User manually edited code.",
                            "code": state["code"],
                            "errors": None,
                        }
                    )
                    with tempfile.TemporaryDirectory() as tmpdir:
                        prefix = Path(tmpdir) / "preview"
                        state["renders"] = render_scenario(state["mjcf"], str(prefix))
                        persist_renders(state)
                    st.success("Re-validation passed!")
                    st.rerun()
                else:
                    errors = report["error_message"]
                    state["errors"] = errors
                    # Add manual edit failure to history
                    state["attempt_history"].append(
                        {
                            "attempt": "Manual Edit (Failed)",
                            "reasoning": "User manually edited code.",
                            "code": state["code"],
                            "errors": errors,
                        }
                    )
                    st.error(f"Re-validation failed: {errors}")
                    st.rerun()
            except Exception as e:
                err_msg = f"Execution error: {e}"
                state["errors"] = err_msg
                state["attempt_history"].append(
                    {
                        "attempt": "Manual Edit (Error)",
                        "reasoning": "User manually edited code.",
                        "code": state["code"],
                        "errors": err_msg,
                    }
                )
                st.error(err_msg)
                st.rerun()

    with col2:
        st.write("### Renderings")
        if state["renders"]:
            for img_path in state["renders"]:
                st.image(img_path)
        else:
            st.info("No renderings available.")

    st.divider()
    col_a, col_b = st.columns(2)
    with col_a:
        if st.button("Approve CAD"):
            state["stage"] = "FINAL"
            st.rerun()
    with col_b:
        if st.button("Reject & Edit Plan"):
            state["stage"] = "PLAN_APPROVAL"
            st.rerun()


def render_final_stage(state):
    st.subheader("4. Final Benchmark Artifact")

    st.success("Benchmark fulfillment complete!")

    tab1, tab2, tab3 = st.tabs(["MJCF XML", "Visuals", "Metadata"])

    with tab1:
        st.code(state["mjcf"], language="xml")
        st.download_button(
            "Download MJCF", state["mjcf"], file_name="scenario.xml", mime="text/xml"
        )

    with tab2:
        if state["renders"]:
            for img_path in state["renders"]:
                st.image(img_path)

    with tab3:
        st.json(
            {
                "request": state["request"],
                "plan": state["plan"],
                "attempts": state["attempts"],
            }
        )

    if st.button("Final Approval & Save"):
        st.balloons()
        st.write("Artifact saved to datasets/benchmarks/ (mocked)")
        # In a real impl, we would call the manager to save properly
