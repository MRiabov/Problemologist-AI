import json
import re
from typing import Any
from pathlib import Path

import dspy
import structlog
import yaml

logger = structlog.get_logger(__name__)


class MockDSPyLM(dspy.LM):
    """Mock DSPy LM using YAML scenarios keyed by session_id prefixes."""

    def __init__(
        self, session_id: str | None = None, node_type: str | None = None, **kwargs
    ):
        super().__init__(model="mock-dspy-model", **kwargs)
        self.session_id = session_id or "default-session"
        self.node_type = node_type
        self.provider = "openai"
        self.responses_path = Path("tests/integration/mock_responses.yaml")
        self.scenarios = self._load_scenarios()
        self._call_counts = {}  # Tracks calls per node key to detect loops

    def _load_scenarios(self) -> dict:
        if not self.responses_path.exists():
            logger.warning(
                "mock_responses_yaml_not_found", path=str(self.responses_path)
            )
            return {}
        try:
            with self.responses_path.open("r") as f:
                data = yaml.safe_load(f)
                return data.get("scenarios", {})
        except Exception as e:
            logger.error("mock_responses_yaml_load_failed", error=str(e))
            return {}

    def __call__(
        self,
        prompt: str | None = None,
        messages: list[dict[str, Any]] | None = None,
        **_kwargs,
    ):
        logger.info(
            "mock_dspy_lm_call",
            session_id=self.session_id,
            explicit_node=self.node_type,
        )

        full_text = self._get_full_text(prompt, messages)

        # 1. Detect Scenario from session_id
        scenario_id = self._get_scenario_id()
        scenario = self.scenarios.get(scenario_id, self.scenarios.get("default", {}))

        # 2. Detect Node Type (Use explicit if set, otherwise detect)
        node_key = self.node_type or self._detect_node_key(full_text)

        # Detect if JSON is requested or if we should provide it for structured nodes
        is_json_requested = (
            "output fields" in full_text.lower() and "json" in full_text.lower()
        )

        # Extract expected fields for ReAct compatibility
        expected_fields = []

        logger.info("mock_dspy_full_text", text=full_text[:500] + "...")

        # Always try to detect fields from text if they exist
        match = re.search(
            r"Expected to find output fields in the LM response: \[([^\]]+)\]",
            full_text,
        )
        if match:
            expected_fields = [f.strip() for f in match.group(1).split(",")]
        else:
            # Detect from role/instructions
            if "next_tool_name" in full_text or "next_thought" in full_text:
                expected_fields.extend(
                    ["next_thought", "next_tool_name", "next_tool_args"]
                )

            for field in [
                "reasoning",
                "thought",
                "summary",
                "review",
                "journal",
                "plan",
                "finished",
                "generated_code",
            ]:
                if field in full_text:
                    expected_fields.append(field)

        expected_fields = list(dict.fromkeys(expected_fields))

        # Force JSON if we have expected fields
        is_json = is_json_requested or len(expected_fields) > 0

        # Normalize benchmark_planner -> planner, but keep electronics_planner as is
        lookup_key = node_key
        if node_key == "benchmark_planner":
            lookup_key = "planner"
        elif node_key == "benchmark_coder":
            lookup_key = "coder"
        elif node_key == "benchmark_reviewer":
            lookup_key = "reviewer"

        node_data = scenario.get(lookup_key, {})
        logger.info(
            "mock_dspy_lm_data_found",
            scenario=scenario_id,
            node=lookup_key,
            has_data=bool(node_data),
        )

        # 3. Handle multi-turn state and loop protection
        count = self._call_counts.get(node_key, 0)
        self._call_counts[node_key] = count + 1

        if count > 5:
            logger.warning("mock_dspy_lm_loop_detected", node=node_key, count=count)
            return self._handle_finish(lookup_key, node_data, is_json, expected_fields)

        if self._is_finishing(full_text):
            return self._handle_finish(lookup_key, node_data, is_json, expected_fields)

        return self._handle_action(lookup_key, node_data, is_json, expected_fields)

    def _get_full_text(
        self, prompt: str | None, messages: list[dict[str, Any]] | None
    ) -> str:
        text = str(prompt or "")
        if messages:
            for msg in messages:
                text += str(msg.get("content", ""))
        return text

    def _get_scenario_id(self) -> str:
        """Extract scenario ID from session_id, handling UUIDs and test prefixes."""
        # Check for UUID (roughly xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx)
        uuid_regex = r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$"
        if re.match(uuid_regex, self.session_id.lower()):
            return "benchmark"

        parts = self.session_id.split("-")
        if len(parts) > 1:
            # If the last part looks like a random hash or short hex (e.g. 'abcd', '1234')
            # and there are at least 2 other parts (e.g. 'INT-002-abcd'), strip it.
            last = parts[-1].lower()
            is_noise = (
                len(last) <= 8 and all(c in "0123456789abcdef" for c in last)
            ) or last.isdigit()
            if is_noise and len(parts) > 2:
                scenario = "-".join(parts[:-1])
                # Normalize INT-002-xyz -> INT-002
                if scenario.startswith("INT-"):
                    return scenario.split("-")[0] + "-" + scenario.split("-")[1]
                return scenario

            # Special case: INT-002 should stay INT-002
            return self.session_id

        return self.session_id

    def _detect_node_key(self, text: str) -> str:
        """Robustly detect node type by looking for role keywords and output fields."""
        low_text = text.lower()

        # 1. Specific Role Headers (Highest Priority)
        if "sidecar learner" in low_text:
            return "skill_learner"
        if "expert designer of spatial" in low_text:
            return "planner"
        if "lead mechanical engineer (planner)" in low_text:
            return "planner"
        if "cad engineer" in low_text or "build123d script" in low_text:
            return "coder"
        if "design reviewer" in low_text or "benchmark auditor" in low_text:
            return "reviewer"
        if "commercial off-the-shelf" in low_text or "cots search" in low_text:
            return "cots_search"
        if "electrical strategy" in low_text or "electronics engineer" in low_text:
            return "electronics_planner"

        # 2. Field-based detection (DSPy standard prompts)
        if "journal" in low_text and (
            "output fields" in low_text or "result:" in low_text
        ):
            return "coder"
        if "review" in low_text and (
            "output fields" in low_text or "result:" in low_text
        ):
            return "reviewer"
        if "plan" in low_text and (
            "output fields" in low_text or "result:" in low_text
        ):
            return "planner"
        if "summary" in low_text and (
            "output fields" in low_text or "result:" in low_text
        ):
            # summary is common in skills/sidecar
            return "skill_learner"
        if "search_summary" in low_text and (
            "output fields" in low_text or "result:" in low_text
        ):
            return "cots_search"

        # 3. Fallbacks by loose role keywords
        if any(kw in low_text for kw in ["skill", "sidecar", "learner"]):
            return "skill_learner"
        if any(kw in low_text for kw in ["reviewer", "critic", "auditor"]):
            return "reviewer"
        if "cots" in low_text or "search" in low_text:
            return "cots_search"
        if any(
            kw in low_text for kw in ["cad engineer", "build123d", "coder", "implement"]
        ):
            return "coder"
        if "planner" in low_text:
            return "planner"
        return "planner"

    def _is_finishing(self, text: str) -> bool:
        low_text = text.lower()
        # dspy.ReAct adds "Observation:" to the trajectory.
        return (
            "stdout:" in low_text or "stderr:" in low_text or "observation:" in low_text
        )

    def _handle_finish(
        self, node_key: str, node_data: dict, is_json: bool, expected_fields: list[str]
    ):
        return self._generate_response(
            node_key, node_data, is_json, finished=True, expected_fields=expected_fields
        )

    def _handle_action(
        self, node_key: str, node_data: dict, is_json: bool, expected_fields: list[str]
    ):
        # Force finish for reviewer to avoid loops in mock
        force_finish = node_key in ["reviewer"]
        return self._generate_response(
            node_key,
            node_data,
            is_json,
            finished=node_data.get("finished", force_finish),
            expected_fields=expected_fields,
        )

    def _generate_response(
        self,
        node_key: str,
        node_data: dict,
        is_json: bool,
        finished: bool,
        expected_fields: list[str] | None = None,
    ):
        """Unified response generator ensures all required fields are present."""
        logger.info("mock_dspy_lm_generate", node=node_key, finished=finished)

        thought = node_data.get("thought", "Task in progress.")
        reasoning = node_data.get("reasoning", "Verified all requirements.")
        expected_fields = expected_fields or []

        # Base schema for DSPy JSONAdapter/CodeAct
        resp = {
            "thought": thought,
            "reasoning": reasoning,
            "finished": finished,
            "generated_code": node_data.get("generated_code", None),
        }

        # ReAct compatibility - handle intermediate tool calling phase
        if "next_tool_name" in expected_fields:
            import shlex

            resp["next_thought"] = thought
            code = node_data.get("generated_code")
            if code and not finished:
                resp["next_tool_name"] = "execute_command"
                # Ensure the python code is executed as a shell command
                resp["next_tool_args"] = {
                    "command": f"python3 -c {shlex.quote(code)}"
                }
            else:
                resp["next_tool_name"] = "finish"
                resp["next_tool_args"] = {}

        # Add node-specific fields (for CodeAct or ReAct extraction phase)
        # Signature fields should be present even in ReAct finish responses
        if node_key == "planner" or node_key == "benchmark_planner":
            resp["plan"] = node_data.get("plan", "No plan provided.")
            resp["summary"] = node_data.get("summary", "Plan generated.")
            # Support BenchmarkPlannerSignature
            if "plan" in node_data and isinstance(node_data["plan"], dict):
                resp["plan"] = node_data["plan"]
        elif (
            node_key == "reviewer"
            or node_key == "plan_reviewer"
            or node_key == "execution_reviewer"
        ):
            resp["review"] = node_data.get(
                "review",
                {
                    "decision": "approved",
                    "reason": "Verified.",
                    "required_fixes": [],
                },
            )
        elif node_key == "coder" or node_key == "benchmark_coder":
            resp["journal"] = node_data.get("journal", "Work completed.")
            if not finished and "generated_code" not in resp:
                resp["generated_code"] = node_data.get("generated_code", "# No code")
        elif node_key == "skill_learner":
            resp["summary"] = node_data.get("summary", "Skills identified.")
            resp["journal"] = node_data.get("journal", "Learning complete.")
        elif node_key == "cots_search":
            resp["search_summary"] = node_data.get("search_summary", "Search complete.")
        elif node_key == "electronics_planner":
            resp["reasoning"] = reasoning
            resp["summary"] = node_data.get("summary", "Electronics plan added.")

        if is_json:
            # Only return fields that were actually requested to avoid Adapter errors
            if expected_fields:
                # Add signature fields to expected_fields if they are present in resp
                # but NOT already in expected_fields.
                # ReAct often asks for [next_thought, next_tool_name, next_tool_args]
                # but then fails if [summary] is missing in the result dict if it's the last turn.
                filtered_resp = {
                    k: v
                    for k, v in resp.items()
                    if k in expected_fields
                    or k
                    in [
                        "reasoning",
                        "thought",
                        "summary",
                        "plan",
                        "review",
                        "journal",
                        "search_summary",
                        "next_thought",
                        "next_tool_name",
                        "next_tool_args"
                    ]
                }
                logger.info("mock_dspy_returning_json", json=filtered_resp)
                return [json.dumps(filtered_resp)]
            return [json.dumps(resp)]

        # Fallback for plain text (rare in our CodeAct setups)
        if node_key == "reviewer":
            return ["Review Result: approved\nReasoning: Approved."]
        return [
            f"Thought: {thought}\nReasoning: {reasoning}\nFinal Answer: Task complete."
        ]
