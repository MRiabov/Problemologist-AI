import streamlit as st
import os
import traceback
import tempfile
from typing import Dict, Any, Optional

from src.generators.benchmark.agent import planner_node, coder_node, validator_node, MAX_ATTEMPTS
from src.generators.benchmark.renderer import render_scenario
from src.dashboard.components.viewer_3d import render_3d_artifact


def render_benchmark_generator():
    st.header("Interactive Benchmark Generator")

    # Initialize state
    if "bg_state" not in st.session_state:
        st.session_state.bg_state = {
            "stage": "INPUT",  # INPUT, PLANNING, PLAN_APPROVAL, CODING, CAD_APPROVAL, FINAL
            "request": "",
            "plan": "",
            "planner_reasoning": "",
            "code": "",
            "coder_reasoning": "",
            "mjcf": "",
            "errors": None,
            "attempts": 0,
            "renders": [],
        }

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
        st.session_state.bg_state = {
            "stage": "INPUT",
            "request": "",
            "plan": "",
            "code": "",
            "mjcf": "",
            "errors": None,
            "attempts": 0,
            "renders": [],
        }
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
    with st.spinner("Generating CAD model and self-validating..."):
        try:
            # Iterative Loop for internal validation
            for i in range(MAX_ATTEMPTS):
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

                # 2. Validate
                val_result = validator_node({"code": state["code"]})

                if val_result.get("validation_passed"):
                    state["mjcf"] = val_result["mjcf"]
                    state["errors"] = None
                    # 3. Render
                    with tempfile.TemporaryDirectory() as tmpdir:
                        prefix = os.path.join(tmpdir, "preview")
                        state["renders"] = render_scenario(state["mjcf"], prefix)
                        # We need to persist these images in session state as bytes or paths
                        # For simplicity, let's just keep paths for now (risky if tmpdir deleted)
                        # Actually, let's copy them to a persistent dashboard folder
                        persist_renders(state)

                    state["stage"] = "CAD_APPROVAL"
                    st.rerun()
                    return
                else:
                    state["errors"] = val_result["errors"]

            st.error(
                f"Failed to generate valid CAD model after {MAX_ATTEMPTS} attempts. Last error: {state['errors']}"
            )
            state["stage"] = "PLAN_APPROVAL"  # Go back to plan
        except Exception as e:
            st.error(f"Coding stage failed: {e}\n{traceback.format_exc()}")
            state["stage"] = "PLAN_APPROVAL"


def persist_renders(state):
    render_dir = os.path.join(".agent_storage", "dashboard_renders")
    os.makedirs(render_dir, exist_ok=True)
    new_paths = []
    for p in state["renders"]:
        basename = os.path.basename(p)
        dest = os.path.join(render_dir, basename)
        import shutil

        shutil.copy(p, dest)
        new_paths.append(dest)
    state["renders"] = new_paths


def render_cad_approval_stage(state):
    st.subheader("3. Review CAD Model & Renderings")

    if state["coder_reasoning"]:
        with st.expander("🧠 View Agent Reasoning", expanded=False):
            st.markdown(state["coder_reasoning"])

    col1, col2 = st.columns([1, 1])

    with col1:
        st.write("### Python Code")
        st.code(state["code"], language="python")

        edited_code = st.text_area(
            "Manual Edits (optional)", value=state["code"], height=200
        )

        if st.button("Re-validate with Edits"):
            state["code"] = edited_code
            val_result = validator_node({"code": state["code"]})
            if val_result.get("validation_passed"):
                state["mjcf"] = val_result["mjcf"]
                state["errors"] = None
                with tempfile.TemporaryDirectory() as tmpdir:
                    prefix = os.path.join(tmpdir, "preview")
                    state["renders"] = render_scenario(state["mjcf"], prefix)
                    persist_renders(state)
                st.success("Re-validation passed!")
                st.rerun()
            else:
                st.error(f"Re-validation failed: {val_result['errors']}")

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
