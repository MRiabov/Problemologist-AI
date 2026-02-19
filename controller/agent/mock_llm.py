import dspy
import structlog
import json
import re
from typing import Any
from langchain_core.messages import BaseMessage, AIMessage
from langchain_core.outputs import ChatResult, ChatGeneration
from langchain_openai import ChatOpenAI

logger = structlog.get_logger(__name__)


class MockChatOpenAI(ChatOpenAI):
    """Mock LangChain ChatOpenAI for integration tests."""

    model_name: str = "mock-model"

    def _generate(
        self, messages: list[BaseMessage], stop: list[str] | None = None, **_kwargs: Any
    ) -> ChatResult:
        logger.info("mock_chat_openai_generate", messages_count=len(messages))
        content = "Understood. I will complete the task."
        # Simple heuristic for common test tasks
        task_text = messages[0].content if messages else ""
        if "box" in task_text.lower() or "cube" in task_text.lower():
            content = "I have successfully created the box as requested."

        message = AIMessage(content=content)
        generation = ChatGeneration(message=message)
        return ChatResult(generations=[generation])


class MockDSPyLM(dspy.LM):
    """Mock DSPy LM for integration tests, specifically supporting CodeAct flow."""

    def __init__(self, **kwargs):
        super().__init__(model="mock-dspy-model", **kwargs)
        self.provider = "openai"

    def __call__(
        self,
        prompt: str | None = None,
        messages: list[dict[str, Any]] | None = None,
        **kwargs,
    ):
        logger.info(
            "mock_dspy_lm_call",
            has_prompt=prompt is not None,
            has_messages=messages is not None,
        )

        # Determine the full text to scan for keywords
        full_text = ""
        if prompt:
            full_text += str(prompt).lower()
        if messages:
            for msg in messages:
                full_text += str(msg.get("content", "")).lower()

        # Check if the prompt suggests structured output (JSON)
        # dspy.Adapters or signatures with Pydantic often use JSON
        is_json_expected = "output fields" in full_text and "json" in full_text

        # Heuristic for INT-002: "Build a simple box of 10x10x10mm."
        if "box" in full_text or "cube" in full_text:
            # If the last message in history is tool output, we should finish
            if "stdout:" in full_text or "stderr:" in full_text:
                logger.info("mock_dspy_lm_finish_task")
                if is_json_expected:
                    # Return composite JSON for Reviewer or structured finish
                    # dspy.CodeAct internal signature expects generated_code and finished
                    response = {
                        "thought": "The box has been created successfully.",
                        "generated_code": None,
                        "finished": True,
                        "review": {
                            "decision": "APPROVE",
                            "reason": "The design satisfies all requirements.",
                            "required_fixes": [],
                        },
                    }
                    return [json.dumps(response)]
                return [
                    "Thought: The box has been created successfully.\nFinal Answer: Task complete."
                ]

            # Helper to create valid yaml/md content
            plan_md = (
                "# Solution Overview\nBuilding a box.\n"
                "# Parts List\n- box: aluminum\n"
                "# Assembly Strategy\nSingle part.\n"
                "# Cost & Weight Budget\nLow.\n"
                "# Risk Assessment\nNone.\n"
            )
            todo_md = "- [x] Step 1\n"
            ass_def = "manufactured_parts: []\nfinal_assembly: []\n"
            obj_yaml = "physics:\n  backend: genesis\n"

            # Combined code to create ALL needed files to satisfy validation
            code = (
                "from build123d import *\n"
                "from shared.models.schemas import PartMetadata\n"
                f"with open('plan.md', 'w') as f: f.write({repr(plan_md)})\n"
                f"with open('todo.md', 'w') as f: f.write({repr(todo_md)})\n"
                f"with open('assembly_definition.yaml', 'w') as f: f.write({repr(ass_def)})\n"
                f"with open('objectives.yaml', 'w') as f: f.write({repr(obj_yaml)})\n"
                "def build():\n"
                "    b = Box(10, 10, 10)\n"
                "    b.label = 'box'\n"
                "    b.metadata = PartMetadata(material_id='aluminum_6061', fixed=True)\n"
                "    return b\n"
                "part = build()\n"
                "export_stl(part, 'box.stl')\n"
                "with open('simulation_result.json', 'w') as f: f.write('{\"success\": true}')\n"
            )

            logger.info("mock_dspy_lm_returning_code_act")
            if is_json_expected:
                response = {
                    "thought": "I will build a 10x10x10mm box and create required documentation.",
                    "generated_code": code,
                    "finished": False,
                }
                return [json.dumps(response)]

            return [
                f"Thought: I will build a 10x10x10mm box and create required documentation.\n"
                f"Action: python\n```python\n{code}```"
            ]

        # Fallback for other nodes/tasks
        if is_json_expected:
            logger.info("mock_dspy_lm_returning_fallback_json")
            response = {
                "thought": "Task complete.",
                "generated_code": None,
                "finished": True,
                "summary": "Task complete.",
                "journal": "Task complete.",
            }
            return [json.dumps(response)]

        logger.info("mock_dspy_lm_returning_default_text")
        return ["Thought: I've processed the request.\nFinal Answer: Task complete."]
