import tempfile
import traceback
from pathlib import Path

import streamlit as st

from src.generators.benchmark.agent import (
    MAX_ATTEMPTS,
    coder_node,
    linter_node,
    planner_node,
    validator_node,
)
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
            result = planner_node({"request": state["request"], "attempts": 0})
            state["plan"] = result["plan"]
            state["planner_reasoning"] = result.get("planner_reasoning", "")
            state["stage"] = "PLAN_APPROVAL"
            st.rerun()
        except Exception as e:
            st.error(f"Planning failed: {e}")
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

    # Clear history for new run if just started
    if state["attempts"] == 0:
        state["attempt_history"] = []

    try:
        # Iterative Loop for internal validation
        for i in range(MAX_ATTEMPTS):
            attempt_idx = i + 1
            if len(state["attempt_history"]) >= attempt_idx:
                continue  # Already processed this attempt in a previous rerun

            status_placeholder.info(f"Running Attempt {attempt_idx}/{MAX_ATTEMPTS}...")

            # 1. Generate Code
            coder_result = coder_node(
                {
                    "request": state["request"],
                    "plan": state["plan"],
                    "code": state.get("code"),
                    "errors": state.get("errors"),
                    "attempts": state["attempts"],
                }
            )
            state["code"] = coder_result["code"]
            state["coder_reasoning"] = coder_result.get("coder_reasoning", "")
            state["attempts"] = coder_result["attempts"]

            current_attempt = {
                "attempt": attempt_idx,
                "reasoning": state["coder_reasoning"],
                "code": state["code"],
                "errors": None,
            }

            # 2. Lint
            status_placeholder.info(f"Linting Attempt {attempt_idx}...")
            lint_result = linter_node({"code": state["code"]})
            if lint_result.get("linting_failed"):
                current_attempt["errors"] = lint_result["errors"]
                state["errors"] = lint_result["errors"]
                state["attempt_history"].append(current_attempt)
                # We let the loop continue to next attempt
                continue

            # 3. Validate
            status_placeholder.info(f"Validating Attempt {attempt_idx} in MuJoCo...")
            val_result = validator_node({"code": state["code"]})

            if val_result.get("validation_passed"):
                state["mjcf"] = val_result["mjcf"]
                state["errors"] = None
                current_attempt["errors"] = None
                state["attempt_history"].append(current_attempt)

                # 4. Render
                status_placeholder.info("Success! Rendering preview...")
                with tempfile.TemporaryDirectory() as tmpdir:
                    prefix = Path(tmpdir) / "preview"
                    state["renders"] = render_scenario(state["mjcf"], str(prefix))
                    persist_renders(state)

                state["stage"] = "CAD_APPROVAL"
                st.rerun()
                return
            state["errors"] = val_result["errors"]
            current_attempt["errors"] = val_result["errors"]
            state["attempt_history"].append(current_attempt)

        # If we reach here, we failed all attempts
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
            val_result = validator_node({"code": state["code"]})
            if val_result.get("validation_passed"):
                state["mjcf"] = val_result["mjcf"]
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
                state["errors"] = val_result["errors"]
                # Add manual edit failure to history
                state["attempt_history"].append(
                    {
                        "attempt": "Manual Edit (Failed)",
                        "reasoning": "User manually edited code.",
                        "code": state["code"],
                        "errors": val_result["errors"],
                    }
                )
                st.error(f"Re-validation failed: {val_result['errors']}")
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
