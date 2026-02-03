import streamlit as st

from src.compiler.models import DashStep


def render_chat(steps: list[DashStep], current_index: int):
    """Renders the agent chat and reasoning history."""
    if not steps:
        st.info("No activity recorded for this episode.")
        return

    # Only show steps up to the currently selected step
    visible_steps = steps[: current_index + 1]

    for step in visible_steps:
        # Agent Role Display
        agent_role = step.agent_role
        role_prefix = f"**[{agent_role}]** " if agent_role else ""

        # User Prompt
        if step.type == "user":
            with st.chat_message("user"):
                st.markdown(step.content or "")

        # Handoff Notification
        elif step.type == "handoff":
            st.info(f"🔄 Handoff: {step.content or 'Switching agents...'}")

        # Agent Thought / Content
        elif step.type == "thought" or step.content:
            with st.chat_message("assistant"):
                st.markdown(f"{role_prefix}{step.content or ''}")

                # Show reasoning trace if available in metadata
                if step.metadata and "reasoning_trace" in step.metadata:
                    with st.expander("Reasoning Trace", expanded=False):
                        st.write(step.metadata["reasoning_trace"])

        # Tool Calls (if any)
        if step.tool_name and step.tool_name != "manual_insert":
            with st.chat_message("assistant"):
                with st.expander(f"🛠️ Tool Call: {step.tool_name}", expanded=False):
                    st.text(step.tool_input or "")

                    if step.tool_output:
                        st.markdown("**Output:**")
                        st.text(step.tool_output)
