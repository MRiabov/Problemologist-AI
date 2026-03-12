"""Deprecated request model module."""


def __getattr__(name: str):
    if name == "AgentRunRequest":
        raise ValueError(
            "deprecated functionality removed: import AgentRunRequest from "
            "controller.api.schemas"
        )
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__: list[str] = []
