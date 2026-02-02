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
        # User Prompt
        if step.get("type") == "user":
            with st.chat_message("user"):
                st.markdown(step["content"])

        # Agent Thought / Content
        elif step.get("type") == "thought" or step.get("content"):
            with st.chat_message("assistant"):
                st.markdown(step["content"])

        # Tool Calls (if any)
        if step.get("tool_calls"):
            tool_call = step["tool_calls"]
            with st.expander(f"🛠️ Tool Call: {tool_call['name']}", expanded=False):
                st.json(tool_call.get("inputs", {}))

                if step.get("tool_output"):
                    st.text_area(
                        "Output",
                        value=str(step["tool_output"]),
                        height=100,
                        disabled=True,
                    )
