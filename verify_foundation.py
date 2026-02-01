import sys
import os

# Add src to path
sys.path.append(os.path.join(os.getcwd(), "src"))

from agent.utils.llm import get_model
from agent.graph.state import AgentState
from agent.tools.env import (
    write_script,
    edit_script,
    preview_design,
    submit_design,
    search_docs,
)
from langchain_core.language_models.chat_models import BaseChatModel


def test_foundation():
    print("Verifying Foundation...")

    # 1. Test get_model factory shells
    print("Testing get_model factory...")
    # Mocking environment keys to avoid errors during instantiation if not present
    os.environ["OPENAI_API_KEY"] = "sk-mock"
    os.environ["GOOGLE_API_KEY"] = "mock-key"
    os.environ["ANTHROPIC_API_KEY"] = "mock-key"

    model = get_model("gpt-4o")
    assert isinstance(model, BaseChatModel)
    print("✓ get_model returns BaseChatModel")

    # 2. Test AgentState
    print("Testing AgentState definition...")
    state: AgentState = {
        "messages": [],
        "plan": "Test plan",
        "step_count": 0,
        "scratchpad": {},
    }
    assert state["plan"] == "Test plan"
    print("✓ AgentState is valid TypedDict")

    # 3. Test Tools binding
    print("Testing tools binding...")
    tools = [write_script, edit_script, preview_design, submit_design, search_docs]
    try:
        model_with_tools = model.bind_tools(tools)
        print("✓ Tools are successfully bindable to model")
    except Exception as e:
        print(f"✗ Tools binding failed: {e}")
        sys.exit(1)

    print("\nAll Foundation checks passed!")


if __name__ == "__main__":
    test_foundation()
