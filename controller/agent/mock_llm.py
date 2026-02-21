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
        is_json = "output fields" in full_text.lower() and "json" in full_text.lower()

        # 1. Detect Scenario from session_id
        scenario_id = self._get_scenario_id()
        scenario = self.scenarios.get(scenario_id, self.scenarios.get("default", {}))

        # 2. Detect Node Type (Use explicit if set, otherwise detect)
        node_key = self.node_type or self._detect_node_key(full_text)

        # Normalize benchmark_planner -> planner, etc. for scenario lookup
        lookup_key = node_key
        if "_" in node_key:
            # benchmark_planner -> planner
            # skill_learner -> skill_learner (kept if matches)
            parts = node_key.split("_")
            if parts[-1] in ["planner", "coder", "reviewer"]:
                lookup_key = parts[-1]

        node_data = scenario.get(lookup_key, {})

        # 3. Handle multi-turn state and loop protection
        count = self._call_counts.get(node_key, 0)
        self._call_counts[node_key] = count + 1

        if count > 5:
            logger.warning("mock_dspy_lm_loop_detected", node=node_key, count=count)
            return self._handle_finish(node_key, node_data, is_json)

        if self._is_finishing(full_text):
            return self._handle_finish(node_key, node_data, is_json)

        return self._handle_action(node_key, node_data, is_json)

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
        return "stdout:" in low_text or "stderr:" in low_text

    def _handle_finish(self, node_key: str, node_data: dict, is_json: bool):
        return self._generate_response(node_key, node_data, is_json, finished=True)

    def _handle_action(self, node_key: str, node_data: dict, is_json: bool):
        # Force finish for reviewer and planner to avoid loops in mock
        force_finish = node_key in ["reviewer", "planner"]
        return self._generate_response(
            node_key,
            node_data,
            is_json,
            finished=node_data.get("finished", force_finish),
        )

    def _generate_response(
        self, node_key: str, node_data: dict, is_json: bool, finished: bool
    ):
        """Unified response generator ensures all required fields are present."""
        logger.info("mock_dspy_lm_generate", node=node_key, finished=finished)

        thought = node_data.get("thought", "Task in progress.")
        reasoning = node_data.get("reasoning", "Verified all requirements.")

        # Base schema for DSPy JSONAdapter/CodeAct
        resp = {
            "thought": thought,
            "reasoning": reasoning,
            "finished": finished,
            "generated_code": node_data.get("generated_code", None),
        }

        # Add node-specific fields
        if node_key == "planner":
            resp["plan"] = node_data.get("plan", "No plan provided.")
            resp["summary"] = node_data.get("summary", "Plan generated.")
        elif node_key == "reviewer":
            resp["review"] = node_data.get(
                "review",
                {
                    "decision": "approved",
                    "reason": "Verified.",
                    "required_fixes": [],
                },
            )
        elif node_key == "coder":
            resp["journal"] = node_data.get("journal", "Work completed.")
            if not finished:
                resp["generated_code"] = node_data.get("generated_code", "# No code")
        elif node_key == "skill_learner":
            resp["summary"] = node_data.get("summary", "Skills identified.")
            resp["journal"] = node_data.get("journal", "Learning complete.")
        elif node_key == "cots_search":
            resp["search_summary"] = node_data.get("search_summary", "Search complete.")

        if is_json:
            return [json.dumps(resp)]

        # Fallback for plain text (rare in our CodeAct setups)
        if node_key == "reviewer":
            return ["Review Result: approved\nReasoning: Approved."]
        return [
            f"Thought: {thought}\nReasoning: {reasoning}\nFinal Answer: Task complete."
        ]
