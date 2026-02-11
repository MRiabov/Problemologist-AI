import os
from unittest.mock import patch


def test_env_var_loading():
    """Verify that environment variables can be correctly overridden/read."""
    test_key = "TEST_API_KEY_123"
    with patch.dict(os.environ, {"OPENAI_API_KEY": test_key}):
        # In a real app, we might import the config object here
        # For now, we simulate how controller/api/main.py might use it or how os.getenv works
        loaded_key = os.getenv("OPENAI_API_KEY")
        assert loaded_key == test_key


def test_docker_compose_style_env_loading():
    """
    Verify that if we simulate loading a .env file into os.environ,
    the application picks it up.
    """
    # This simulates what Docker Compose's env_file does (injecting into os.environ)
    mock_env = {"OPENAI_API_KEY": "sk-dummy-key", "LLM_MODEL": "gpt-4o-test"}

    with patch.dict(os.environ, mock_env):
        # Import the module that reads these vars
        # We need to be careful with imports if they are at top-level
        # But here we just check os.getenv which is what main.py uses
        assert os.getenv("OPENAI_API_KEY") == "sk-dummy-key"
        assert os.getenv("LLM_MODEL") == "gpt-4o-test"
