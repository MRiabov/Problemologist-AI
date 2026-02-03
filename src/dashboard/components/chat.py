from typing import Any

import streamlit as st


def render_chat(steps: list[dict[str, Any]], current_index: int):
    """Renders the agent chat and reasoning history."""
    if not steps:
        st.info("No activity recorded for this episode.")
        return

    # Only show steps up to the currently selected step
    visible_steps = steps[: current_index + 1]

    for step in visible_steps:
        # Agent Role Display
        agent_role = step.get("agent_role")
        role_prefix = f"**[{agent_role}]** " if agent_role else ""

        # User Prompt
        if step.get("type") == "user":
            with st.chat_message("user"):
                st.markdown(step.get("content") or step.get("tool_input") or "")

        # Handoff Notification
        elif step.get("type") == "handoff":
            st.info(f"🔄 Handoff: {step.get('content') or 'Switching agents...'}")

        # Agent Thought / Content
        elif step.get("type") == "thought" or step.get("content"):
            with st.chat_message("assistant"):
                st.markdown(f"{role_prefix}{step.get('content') or ''}")

                # Show reasoning trace if available in metadata
                if step.get("metadata") and "reasoning_trace" in step["metadata"]:
                    with st.expander("Reasoning Trace", expanded=False):
                        st.write(step["metadata"]["reasoning_trace"])

        # Tool Calls (if any)
        if step.get("tool_name") and step.get("tool_name") != "manual_insert":
            with st.chat_message("assistant"):
                with st.expander(f"🛠️ Tool Call: {step['tool_name']}", expanded=False):
                    st.text(step.get("tool_input", ""))

                    if step.get("tool_output"):
                        st.markdown("**Output:**")
                        st.text(step["tool_output"])
