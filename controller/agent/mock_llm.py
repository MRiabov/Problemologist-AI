import json
import re
from pathlib import Path
from typing import Any

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
        self._tool_progress = {}  # Tracks completed tool steps from trajectory text
        # Tracks repeated LM invocations without tool-observation progress.
        self._stall_counts = {}

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
            logger.warning("mock_responses_yaml_load_failed", error=str(e))
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
        logger.info("mock_dspy_full_text", text=full_text)

        # 1. Detect scenario.
        # Prefer explicit session_id-mapped scenarios (e.g. INT-014-xxxx -> INT-014)
        # to avoid generic text heuristics accidentally selecting "benchmark".
        scenario_id = self._get_scenario_id()
        if scenario_id not in self.scenarios or scenario_id in {"benchmark", "default"}:
            detected = self._detect_scenario_from_text(full_text)
            if detected:
                scenario_id = detected

        scenario = self.scenarios.get(scenario_id, self.scenarios.get("default", {}))

        # 2. Detect Node Type (Use explicit if set, otherwise detect)
        node_key = self.node_type or self._detect_node_key(full_text)

        # Detect if JSON is requested or if we should provide it for structured nodes
        low_text = full_text.lower()
        is_json_requested = (
            ("output fields" in low_text or "outputs will be" in low_text)
            and "json" in low_text
        ) or "respond with a json object" in low_text

        # Extract expected fields for ReAct compatibility
        expected_fields = []

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

        # WP10: Detect expected fields from JSON template if empty
        if is_json_requested and not expected_fields:
            expected_fields = self._detect_expected_fields_from_json(full_text)
            logger.info("mock_dspy_detected_fields_from_json", fields=expected_fields)

        # Standard ReAct with TypedPredictor uses field-based formatting [[ ## field ## ]]
        is_json = is_json_requested

        logger.info(
            "mock_dspy_expected_fields", fields=expected_fields, is_json=is_json
        )

        # Normalize benchmark_planner -> planner, but keep electronics_planner as is
        lookup_key = node_key
        sig_lookup = node_key
        if node_key == "benchmark_planner":
            lookup_key = "planner"
            sig_lookup = "planner"
        elif node_key == "benchmark_coder":
            lookup_key = "coder"
            sig_lookup = "coder"
        elif node_key == "benchmark_reviewer":
            lookup_key = "reviewer"
            sig_lookup = "reviewer"
        elif node_key in {
            "plan_reviewer",
            "execution_reviewer",
            "electronics_reviewer",
        }:
            lookup_key = "reviewer"
            sig_lookup = "reviewer"

        node_data = scenario.get(lookup_key, {})
        logger.info(
            "mock_dspy_lm_data_found",
            scenario=scenario_id,
            node=lookup_key,
            has_data=bool(node_data),
        )

        # Track explicit tool progress from trajectory markers, not LM call count.
        # DSPy can invoke LM multiple times per step, so call count alone is unstable.
        # We count both the field-based markers and the standard ReAct Observation markers.
        custom_markers = re.findall(
            r"\[\[\s*##\s*observation_\d+\s*##\s*\]\]", full_text
        )
        standard_markers = re.findall(r"Observation:", full_text, re.IGNORECASE)
        completed_tools = len(custom_markers) + len(standard_markers)
        self._tool_progress[lookup_key] = completed_tools

        # 3. Handle multi-turn state and loop protection
        count = self._call_counts.get(lookup_key, 0)
        self._call_counts[lookup_key] = count + 1
        call_index = count + 1

        # Optional assertions for integration hardening.
        self._validate_expected_llm_inputs(
            node_key=lookup_key,
            node_data=node_data,
            full_text=full_text,
            call_index=call_index,
        )
        self._validate_expected_tool_calls(
            node_key=lookup_key,
            node_data=node_data,
            full_text=full_text,
        )
        self._validate_expected_tool_outputs(
            node_key=lookup_key,
            node_data=node_data,
            full_text=full_text,
        )

        # For scripted tool sequences, detect true no-progress stalls.
        # DSPy can invoke LM multiple times per step, so raw call count is insufficient.
        tool_calls = node_data.get("tool_calls", [])
        prev_progress = self._tool_progress.get(f"{lookup_key}__prev", -1)
        stalled_calls = self._stall_counts.get(lookup_key, 0)
        if completed_tools > prev_progress:
            stalled_calls = 0
        else:
            stalled_calls += 1
        self._stall_counts[lookup_key] = stalled_calls
        self._tool_progress[f"{lookup_key}__prev"] = completed_tools

        if stalled_calls > 5 and (not tool_calls or completed_tools >= len(tool_calls)):
            logger.error(
                "mock_dspy_lm_loop_detected",
                node=lookup_key,
                stalled_calls=stalled_calls,
                call_count=call_index,
                completed_tools=completed_tools,
                expected_tool_calls=len(tool_calls),
            )
            raise ValueError(
                f"MockDSPyLM loop detected for node={lookup_key}: "
                f"no tool-progress for {stalled_calls} calls "
                f"(completed_tools={completed_tools}, expected_tool_calls={len(tool_calls)})."
            )

        # Only force finish on observation when explicit tool calls are exhausted.
        tool_calls = node_data.get("tool_calls", [])
        tool_progress = self._tool_progress.get(lookup_key, 0)
        if self._is_finishing(full_text) and (
            not tool_calls or tool_progress >= len(tool_calls)
        ):
            return self._handle_finish(sig_lookup, node_data, is_json, expected_fields)

        return self._handle_action(sig_lookup, node_data, is_json, expected_fields)

    def _detect_expected_fields_from_json(self, text: str) -> list[str]:
        """Detect expected JSON keys from a template in the prompt."""
        # Only look in the last part of the prompt where the output template is
        search_text = text[-2000:] if len(text) > 2000 else text

        # Find blocks that look like JSON templates: { "key": "{key}", ... }
        # We look for "key": and extract the key.
        keys = re.findall(r'"([^"]+)"\s*:', search_text)

        logger.debug("mock_dspy_all_json_keys_found", keys=keys)

        # Filter out common false positives if needed, but for now just take unique ones
        whitelist = [
            "next_thought",
            "next_tool_name",
            "next_tool_args",
            "reasoning",
            "thought",
            "summary",
            "plan",
            "review",
            "journal",
            "summarized_journal",
            "answer",
        ]
        return [k for k in list(dict.fromkeys(keys)) if k in whitelist]

    def _get_full_text(
        self, prompt: str | None, messages: list[dict[str, Any]] | None
    ) -> str:
        text = str(prompt or "")
        if messages:
            for msg in messages:
                text += str(msg.get("content", ""))
        return text

    def _detect_scenario_from_text(self, text: str) -> str | None:
        """Detect scenario ID from prompt text keywords."""
        # Sort by length descending to match most specific first (e.g. INT-177 before INT)
        sorted_scenarios = sorted(self.scenarios.keys(), key=len, reverse=True)
        for scenario_id in sorted_scenarios:
            if scenario_id == "default" or scenario_id == "benchmark":
                continue
            # Use word boundaries for better matching
            if re.search(rf"\b{re.escape(scenario_id)}\b", text):
                return scenario_id

        # Secondary check for benchmark scenario if it looks like one
        if (
            "move" in text.lower() and "sphere" in text.lower()
        ) or "steel ball" in text.lower():
            return "benchmark"

        return None

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
        if "summarizer node" in low_text:
            return "summarizer"
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
            "stdout:" in low_text
            or "stderr:" in low_text
            or "observation:" in low_text
            or "past interactions:" in low_text
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

        count = self._call_counts.get(node_key, 0)
        tool_calls = node_data.get("tool_calls", [])

        # When explicit tool calls exist, continue until all calls are exhausted.
        if tool_calls:
            tool_progress = self._tool_progress.get(node_key, 0)
            should_finish = tool_progress >= len(tool_calls)
        else:
            should_finish = node_data.get("finished", force_finish) or count > 1

        return self._generate_response(
            node_key,
            node_data,
            is_json,
            finished=should_finish,
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

        # WP10: Support explicit tool calls in scenarios
        # We use call_count to track which tool we are on
        count = self._call_counts.get(node_key, 0)
        tool_progress = self._tool_progress.get(node_key, 0)
        tool_calls = node_data.get("tool_calls", [])

        tool_name = None
        tool_args = {}

        if not finished:
            if tool_progress < len(tool_calls) and len(tool_calls) > 0:
                # Still have explicit tools to call.
                tc = tool_calls[tool_progress]
                thought = tc.get("thought", thought)
                tool_name = tc.get("name")
                tool_args = tc.get("input", {})
            else:
                # Explicit tools exhausted, use generated_code if present
                code = node_data.get("generated_code")
                if code:
                    tool_name = "execute_command"
                    tool_args = {"command": code}
                else:
                    tool_name = "finish"
                    tool_args = {}

        # Base schema for DSPy JSONAdapter
        resp = {
            "thought": thought,
            "reasoning": reasoning,
            "finished": finished,
            "generated_code": node_data.get("generated_code"),
        }

        # ReAct compatibility - handle intermediate tool calling phase
        # WP10: Always include these in JSON mode to be safe, as dspy adapters often expect them
        if "next_tool_name" in expected_fields or is_json:
            resp["next_thought"] = thought
            if tool_name:
                resp["next_tool_name"] = tool_name
                resp["next_tool_args"] = tool_args
            else:
                # If finished or no tools left, finish
                resp["next_tool_name"] = "finish"
                resp["next_tool_args"] = {}

        # Add node-specific fields (for ReAct extraction phase)
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
            or node_key == "electronics_reviewer"
        ):
            resp["review"] = node_data.get(
                "review",
                {
                    "decision": "APPROVED",
                    "reason": "Verified.",
                    "required_fixes": [],
                },
            )
        elif node_key == "coder" or node_key == "benchmark_coder":
            resp["journal"] = node_data.get("journal", "Work completed.")
            if not finished and "generated_code" not in resp:
                resp["generated_code"] = node_data.get("generated_code", "# No code")
        elif node_key == "summarizer":
            resp["summarized_journal"] = node_data.get(
                "summarized_journal", "Journal summary."
            )
        elif node_key == "skill_learner":
            resp["summary"] = node_data.get("summary", "Skills identified.")
            resp["journal"] = node_data.get("journal", "Learning complete.")
        elif node_key == "cots_search":
            resp["search_summary"] = node_data.get("search_summary", "Search complete.")
        elif node_key == "electronics_planner":
            resp["reasoning"] = reasoning
            resp["summary"] = node_data.get("summary", "Electronics plan added.")

        if is_json:
            # WP10: Do NOT filter JSON responses. dspy adapters are very sensitive to missing fields
            # but generally ignore extra ones. Filtering often causes AdapterParseError.
            logger.info("mock_dspy_returning_json", json=resp)
            return [json.dumps(resp)]

        # Fallback for field-based format (common in non-JSON dspy.ReAct)
        if expected_fields:
            # Ensure signature-defined output fields are also included
            # ReAct sometimes expects them even in intermediate turns
            sig_lookup = node_key
            if sig_lookup == "benchmark_planner":
                sig_lookup = "planner"
            if sig_lookup == "benchmark_coder":
                sig_lookup = "coder"
            if sig_lookup == "benchmark_reviewer":
                sig_lookup = "reviewer"

            sig_fields = {
                "planner": ["reasoning", "plan", "summary"],
                "coder": ["journal"],
                "reviewer": ["review"],
                "summarizer": ["summarized_journal"],
                "skill_learner": ["summary", "journal"],
                "cots_search": ["search_summary"],
                "electronics_planner": ["reasoning", "summary"],
            }.get(sig_lookup, [])

            # For ReAct tool-call turns, return only the explicitly requested fields.
            # Extra signature fields here can cause parse/retry loops before tool execution.
            if "next_tool_name" in expected_fields:
                all_fields = list(dict.fromkeys(expected_fields))
            else:
                all_fields = list(dict.fromkeys(expected_fields + sig_fields))

            # WP10: Force plan inclusion for planner nodes to ensure state transitions work
            if sig_lookup == "planner" and "plan" not in all_fields:
                all_fields.append("plan")

            lines = []
            for field in all_fields:
                val = resp.get(field)
                if val is None:
                    # Provide default values for missing expected fields to satisfy parser
                    if field == "next_tool_name":
                        val = "finish"
                    elif field == "next_tool_args":
                        val = {}
                    elif field == "plan" and sig_lookup == "planner":
                        val = node_data.get("plan", {})
                    elif field == "review" and sig_lookup == "reviewer":
                        val = node_data.get("review", {})
                    else:
                        val = "None"

                if isinstance(val, bool):
                    val = str(val).lower()
                elif field == "plan" and sig_lookup == "planner":
                    # WP10: Always use JSON for the plan object in planner nodes
                    val = json.dumps(val)
                elif isinstance(val, (dict, list)):
                    val = json.dumps(val)

                # TypedPredictor expects [[ ## field ## ]] followed by value on NEXT line
                lines.append(f"[[ ## {field} ## ]]\n{val!s}")

            # Keep the expected ReAct delimiter to avoid parser retries.
            lines.append("[[ ## completed ## ]]\n")

            result = "\n\n".join(lines)
            logger.info("mock_dspy_returning_fields", text=result)
            return [result]

        # Fallback for plain text (rare in our ReAct setups)
        if node_key == "reviewer":
            return ["Review Result: approved\nReasoning: Approved."]
        return [
            f"Thought: {thought}\nReasoning: {reasoning}\nFinal Answer: Task complete."
        ]

    @staticmethod
    def _normalize_text_list(value: Any) -> list[str]:
        if value is None:
            return []
        if isinstance(value, str):
            return [value]
        if isinstance(value, list):
            return [str(v) for v in value]
        return [str(value)]

    def _assert_text_expectations(
        self,
        *,
        context: str,
        text: str,
        contains: list[str],
        not_contains: list[str],
    ) -> None:
        for token in contains:
            if token not in text:
                raise AssertionError(
                    f"{context}: expected token not found in LM input/trajectory: {token!r}"
                )
        for token in not_contains:
            if token in text:
                raise AssertionError(
                    f"{context}: unexpected token found in LM input/trajectory: {token!r}"
                )

    def _validate_expected_llm_inputs(
        self,
        *,
        node_key: str,
        node_data: dict[str, Any],
        full_text: str,
        call_index: int,
    ) -> None:
        expected = node_data.get("expected_llm_inputs")
        if not expected:
            return

        if isinstance(expected, dict):
            contains = self._normalize_text_list(expected.get("contains"))
            not_contains = self._normalize_text_list(expected.get("not_contains"))
            self._assert_text_expectations(
                context=f"{node_key} call {call_index} expected_llm_inputs",
                text=full_text,
                contains=contains,
                not_contains=not_contains,
            )
            for rule in expected.get("per_call", []):
                if int(rule.get("call_index", -1)) != call_index:
                    continue
                self._assert_text_expectations(
                    context=f"{node_key} call {call_index} expected_llm_inputs.per_call",
                    text=full_text,
                    contains=self._normalize_text_list(rule.get("contains")),
                    not_contains=self._normalize_text_list(rule.get("not_contains")),
                )
            return

        if isinstance(expected, list):
            # If entries are strings, all are required for every call.
            if all(isinstance(item, str) for item in expected):
                self._assert_text_expectations(
                    context=f"{node_key} call {call_index} expected_llm_inputs",
                    text=full_text,
                    contains=[str(item) for item in expected],
                    not_contains=[],
                )
                return

            for rule in expected:
                if not isinstance(rule, dict):
                    continue
                expected_call = rule.get("call_index")
                if expected_call is not None and int(expected_call) != call_index:
                    continue
                self._assert_text_expectations(
                    context=f"{node_key} call {call_index} expected_llm_inputs",
                    text=full_text,
                    contains=self._normalize_text_list(rule.get("contains")),
                    not_contains=self._normalize_text_list(rule.get("not_contains")),
                )

    def _extract_indexed_fields(
        self, full_text: str, field_prefix: str
    ) -> dict[int, str]:
        out: dict[int, str] = {}

        marker_pattern = re.compile(
            rf"\[\[\s*##\s*{re.escape(field_prefix)}_(\d+)\s*##\s*\]\]\s*(.*?)(?=\n\s*\[\[\s*##|\Z)",
            re.IGNORECASE | re.DOTALL,
        )
        for match in marker_pattern.finditer(full_text):
            idx = int(match.group(1))
            value = match.group(2).strip()
            out[idx] = value

        # Fallback parser for non-marker prompt formats.
        kv_pattern = re.compile(
            rf"[\"']?{re.escape(field_prefix)}_(\d+)[\"']?\s*[:=]\s*([^\n]+)",
            re.IGNORECASE,
        )
        for match in kv_pattern.finditer(full_text):
            idx = int(match.group(1))
            out.setdefault(idx, match.group(2).strip().strip(","))

        return out

    def _validate_expected_tool_calls(
        self, *, node_key: str, node_data: dict[str, Any], full_text: str
    ) -> None:
        expected_calls = node_data.get("expected_tool_calls")
        if not expected_calls:
            return

        observed_names = self._extract_indexed_fields(full_text, "tool_name")
        observed_args = self._extract_indexed_fields(full_text, "tool_args")
        max_observed_idx = max(observed_names.keys(), default=-1)

        for idx in range(min(len(expected_calls), max_observed_idx + 1)):
            rule = expected_calls[idx]
            if not isinstance(rule, dict):
                continue

            expected_name = rule.get("name")
            observed_name = observed_names.get(idx)
            if expected_name and observed_name and expected_name != observed_name:
                raise AssertionError(
                    f"{node_key} tool {idx}: expected name {expected_name!r}, got {observed_name!r}"
                )

            args_text = observed_args.get(idx, "")
            self._assert_text_expectations(
                context=f"{node_key} tool {idx} expected_tool_calls",
                text=args_text,
                contains=self._normalize_text_list(rule.get("args_contains")),
                not_contains=self._normalize_text_list(rule.get("args_not_contains")),
            )

    def _validate_expected_tool_outputs(
        self, *, node_key: str, node_data: dict[str, Any], full_text: str
    ) -> None:
        expected_outputs = node_data.get("expected_tool_outputs")
        if not expected_outputs:
            return

        observed_outputs = self._extract_indexed_fields(full_text, "observation")
        max_observed_idx = max(observed_outputs.keys(), default=-1)

        for idx in range(min(len(expected_outputs), max_observed_idx + 1)):
            rule = expected_outputs[idx]
            if isinstance(rule, str):
                contains = [rule]
                not_contains: list[str] = []
            elif isinstance(rule, dict):
                contains = self._normalize_text_list(rule.get("contains"))
                not_contains = self._normalize_text_list(rule.get("not_contains"))
            else:
                continue

            self._assert_text_expectations(
                context=f"{node_key} observation {idx} expected_tool_outputs",
                text=observed_outputs.get(idx, ""),
                contains=contains,
                not_contains=not_contains,
            )
