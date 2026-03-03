from typing import Any

import dspy
import structlog

logger = structlog.get_logger(__name__)


class FirstTurnFullInputReAct(dspy.ReAct):
    """
    ReAct variant that sends full input fields only on the first LM turn.

    On subsequent tool turns, it replaces input fields with compact placeholders
    while preserving the same keys required by the signature.
    """

    @staticmethod
    def _compact_inputs(input_args: dict[str, Any]) -> dict[str, Any]:
        compacted: dict[str, Any] = {}
        for key, value in input_args.items():
            if isinstance(value, str):
                compacted[key] = (
                    f"[context elided after first turn: field={key}, "
                    f"chars={len(value)}]"
                )
            else:
                compacted[key] = value
        return compacted

    def forward(self, **input_args):
        trajectory = {}
        max_iters = input_args.pop("max_iters", self.max_iters)
        first_turn_args = dict(input_args)
        followup_args = self._compact_inputs(first_turn_args)
        current_args = first_turn_args

        for idx in range(max_iters):
            try:
                pred = self._call_with_potential_trajectory_truncation(
                    self.react, trajectory, **current_args
                )
            except ValueError as err:
                logger.warning(
                    "react_invalid_tool_selection",
                    error=str(err),
                    iteration=idx,
                )
                break

            trajectory[f"thought_{idx}"] = pred.next_thought
            trajectory[f"tool_name_{idx}"] = pred.next_tool_name
            trajectory[f"tool_args_{idx}"] = pred.next_tool_args

            try:
                trajectory[f"observation_{idx}"] = self.tools[pred.next_tool_name](
                    **pred.next_tool_args
                )
            except Exception as err:
                trajectory[f"observation_{idx}"] = (
                    f"Execution error in {pred.next_tool_name}: {err}"
                )

            if idx == 0:
                logger.info(
                    "react_followup_inputs_compacted",
                    fields=list(followup_args.keys()),
                )
                current_args = followup_args

            if pred.next_tool_name == "finish":
                break

        extract = self._call_with_potential_trajectory_truncation(
            self.extract, trajectory, **first_turn_args
        )
        return dspy.Prediction(trajectory=trajectory, **extract)

    async def aforward(self, **input_args):
        trajectory = {}
        max_iters = input_args.pop("max_iters", self.max_iters)
        first_turn_args = dict(input_args)
        followup_args = self._compact_inputs(first_turn_args)
        current_args = first_turn_args

        for idx in range(max_iters):
            try:
                pred = await self._async_call_with_potential_trajectory_truncation(
                    self.react, trajectory, **current_args
                )
            except ValueError as err:
                logger.warning(
                    "react_invalid_tool_selection_async",
                    error=str(err),
                    iteration=idx,
                )
                break

            trajectory[f"thought_{idx}"] = pred.next_thought
            trajectory[f"tool_name_{idx}"] = pred.next_tool_name
            trajectory[f"tool_args_{idx}"] = pred.next_tool_args

            try:
                trajectory[f"observation_{idx}"] = await self.tools[
                    pred.next_tool_name
                ].acall(**pred.next_tool_args)
            except Exception as err:
                trajectory[f"observation_{idx}"] = (
                    f"Execution error in {pred.next_tool_name}: {err}"
                )

            if idx == 0:
                logger.info(
                    "react_followup_inputs_compacted_async",
                    fields=list(followup_args.keys()),
                )
                current_args = followup_args

            if pred.next_tool_name == "finish":
                break

        extract = await self._async_call_with_potential_trajectory_truncation(
            self.extract, trajectory, **first_turn_args
        )
        return dspy.Prediction(trajectory=trajectory, **extract)
