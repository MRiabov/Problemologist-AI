import streamlit as st

from src.dashboard.models import DashStep


def render_code(step: DashStep):
    """Renders the code generated or modified in the current step."""
    code_content = None
    filename = "script.py"

    # Strategy 1: Look for code in metadata or tool_input
    if step.tool_name in ["write_file", "edit_file", "write_script"]:
        # Look for content in tool_input or metadata
        code_content = step.metadata.get("content")
        filename = step.metadata.get("path", "script.py")

    # Strategy 2: Look for markdown code blocks in content
    if not code_content and step.content:
        content = step.content
        if "```python" in content:
            code_content = content.split("```python")[1].split("```")[0]
            filename = "Inferred from chat"

    if code_content:
        st.caption(f"File: {filename}")
        st.code(code_content, language="python")
    else:
        st.info("No code changes in this step.")
