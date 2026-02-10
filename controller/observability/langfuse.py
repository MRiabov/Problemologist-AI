import os

from langfuse import Langfuse
from langfuse.langchain import CallbackHandler


def get_langfuse_callback() -> CallbackHandler | None:
    """
    Initialize and return a Langfuse CallbackHandler if credentials are provided.
    """
    public_key = os.getenv("LANGFUSE_PUBLIC_KEY")
    secret_key = os.getenv("LANGFUSE_SECRET_KEY")
    host = os.getenv("LANGFUSE_HOST", "http://localhost:3000")

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
