try:
    import langfuse
    print(f"Langfuse file: {langfuse.__file__}")
    print(f"Langfuse dir: {dir(langfuse)}")
    from langfuse.callback import CallbackHandler
    print("Successfully imported CallbackHandler")
except Exception as e:
    print(f"Error: {e}")
    import sys
    print(f"Path: {sys.path}")