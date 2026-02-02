from typing import Any

import streamlit as st


def render_code(step: dict[str, Any]):
    """Renders the code generated or modified in the current step."""
    code_content = None

    # Strategy 1: Look for code in tool_calls (e.g., write_file)
    tool_calls = step.get("tool_calls")
    if (
        tool_calls
        and isinstance(tool_calls, dict)
        and tool_calls.get("name") in ["write_file", "edit_file", "write_script"]
    ):
        inputs = tool_calls.get("inputs", {})
        code_content = inputs.get("content")
        filename = inputs.get("path", "script.py")

    # Strategy 2: Look for markdown code blocks in content
    if not code_content and step.get("content"):
        content = step["content"]
        if "```python" in content:
            code_content = content.split("```python")[1].split("```")[0]
            filename = "Inferred from chat"

    if code_content:
        st.caption(f"File: {filename}")
        st.code(code_content, language="python")
    else:
        st.info("No code changes in this step.")
