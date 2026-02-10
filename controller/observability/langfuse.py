import os
import uuid
from typing import Any

from langfuse import Langfuse
from langfuse.langchain import CallbackHandler


def get_langfuse_callback(trace_id: str | None = None, name: str | None = None) -> CallbackHandler | None:
    """
    Initialize and return a Langfuse CallbackHandler if credentials are provided.
    Compatible with Langfuse v3+ using trace_context.
    """
    public_key = os.getenv("LANGFUSE_PUBLIC_KEY")
    secret_key = os.getenv("LANGFUSE_SECRET_KEY")
    host = os.getenv("LANGFUSE_HOST", "https://cloud.langfuse.com")

    if public_key and secret_key:
        return CallbackHandler()
    return None


def get_langfuse_client() -> Langfuse | None:
    """
    Initialize and return a Langfuse client if credentials are provided.
    """
    public_key = os.getenv("LANGFUSE_PUBLIC_KEY")
    secret_key = os.getenv("LANGFUSE_SECRET_KEY")
    host = os.getenv("LANGFUSE_HOST", "http://localhost:3000")

    if public_key and secret_key:
        return Langfuse(public_key=public_key, secret_key=secret_key, host=host)
    return None
