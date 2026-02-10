import os

from langfuse.langchain import CallbackHandler

# Note: langfuse.decorators.observe can be used for non-LangChain functions
# Example: from langfuse.decorators import observe


def get_trace_callback() -> CallbackHandler | None:
    """
    Returns a configured LangFuse CallbackHandler if environment variables are set.

    Required environment variables:
    - LANGFUSE_PUBLIC_KEY
    - LANGFUSE_SECRET_KEY

    Optional environment variables:
    - LANGFUSE_HOST (default: https://cloud.langfuse.com)
    """
    public_key = os.getenv("LANGFUSE_PUBLIC_KEY")
    secret_key = os.getenv("LANGFUSE_SECRET_KEY")
    if not public_key or not secret_key:
        # Gracefully handle missing configuration
        return None

    return CallbackHandler()


# The @observe() decorator can be used for tracing non-LangChain functions.
# Example:
# @observe()
# def some_logic():
#     pass
