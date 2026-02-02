import tempfile
import traceback
from pathlib import Path

import streamlit as st

from langchain_core.messages import HumanMessage
from src.agent.graph.nodes.planner import planner_node
from src.generators.benchmark.agent import DEFAULT_RUNTIME_CONFIG, generator_agent
from src.generators.benchmark.renderer import render_scenario


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
            # Extract reasoning if available (not explicitly returned by new planner_node, but could be in messages)
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
        "The plan includes test objectives, rough geometry, and self-collision verification strategy."
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
    status_placeholder = st.empty()
    status_placeholder.info("Running DeepAgents Graph...")

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

        # Invoke the graph
        # This runs until completion or recursion limit
        final_state = generator_agent.invoke(
            input_state, config={"recursion_limit": 50}
        )

        # Analyze results from final_state
        # We need to extract the last code, MJCF, and status
        messages = final_state["messages"]

        # Check for success
        # The actor logs success or the critic does?
        # Ideally we look for ToolMessage from validate_benchmark_model

        last_mjcf = ""
        last_code = ""
        errors = None

        # Backwards scan for validation
        for msg in reversed(messages):
            if hasattr(msg, "tool_calls") and msg.tool_calls:
                # Check if this was a validation call
                pass
            if hasattr(msg, "content"):
                if "Validation Passed!" in str(msg.content):
                    # Found success output from tool
                    # Extract MJCF
                    import re

                    match = re.search(
                        r"MJCF Output.*:\n(<mujoco.*)", str(msg.content), re.DOTALL
                    )
                    if match:
                        last_mjcf = match.group(1)
                    else:
                        # Maybe full content is mjcf?
                        if "<mujoco" in str(msg.content):
                            last_mjcf = str(msg.content)
                    break

        # Extract code (find last write_script call or code block?)
        # DeepAgents actor uses write_script tool? Or just outputs code?
        # New actor uses tools. `validate_benchmark_model` takes code as arg.
        # So we can find the `validate_benchmark_model` call in messages.
        for msg in reversed(messages):
            if hasattr(msg, "tool_calls"):
                for tc in msg.tool_calls:
                    if tc["name"] == "validate_benchmark_model":
                        last_code = tc["args"].get("code", "")
                        break
            if last_code:
                break

        state["code"] = last_code
        state["mjcf"] = last_mjcf
        state["attempts"] = final_state.get("step_count", 0)  # Approximation

        if last_mjcf:
            state["errors"] = None
            status_placeholder.success("Generation Successful!")

            # Render
            with tempfile.TemporaryDirectory() as tmpdir:
                prefix = Path(tmpdir) / "preview"
                state["renders"] = render_scenario(state["mjcf"], str(prefix))
                persist_renders(state)

            state["stage"] = "CAD_APPROVAL"
            st.rerun()
        else:
            state["errors"] = "Generation failed to produce valid MJCF within limits."
            state["stage"] = "CAD_APPROVAL"
            st.rerun()

    except Exception as e:
        st.error(f"Coding stage failed: {e}\n{traceback.format_exc()}")
        state["stage"] = "PLAN_APPROVAL"


def persist_renders(state):
    render_dir = Path(".agent_storage") / "dashboard_renders"
    render_dir.mkdir(parents=True, exist_ok=True)
    new_paths = []
    for p in state["renders"]:
        basename = Path(p).name
        dest = render_dir / basename
        import shutil

        shutil.copy(str(p), str(dest))
        new_paths.append(str(dest))
    state["renders"] = new_paths


from src.generators.benchmark.manager import execute_build
from src.generators.benchmark.validator import validate_mjcf


def render_cad_approval_stage(state):
    st.subheader("4. Review CAD Model & Debug Trace")

    if state["errors"]:
        st.error(
            f"Validation failed after {state['attempts']} attempts. Last error: {state['errors']}"
        )
    else:
        st.success(f"Validation passed after {state['attempts']} attempts!")

    # 1. Debug History
    with st.expander("🧠 Debug: Agent Trace & History", expanded=True):
        st.write("### Planner Reasoning")
        st.markdown(state["planner_reasoning"] or "No reasoning provided.")
        st.divider()

        st.write("### Coder Attempt History")
        for i, attempt in enumerate(state["attempt_history"]):
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
