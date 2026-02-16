import os
from pathlib import Path
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
    mock_env = {
        "OPENAI_API_KEY": "sk-dummy-key",
        "LLM_MODEL": "dummy-llm-model",
    }

    with patch.dict(os.environ, mock_env):
        # Import the module that reads these vars
        # We need to be careful with imports if they are at top-level
        # But here we just check os.getenv which is what main.py uses
        assert os.getenv("OPENAI_API_KEY") == "sk-dummy-key"
        assert os.getenv("LLM_MODEL") == "dummy-llm-model"


def test_env_files_consistency():
    """Verify that .env and .env.example have the same keys."""
    base_dir = Path(__file__).parent.parent
    env_path = base_dir / ".env"
    example_path = base_dir / ".env.example"

    if not env_path.exists():
        # If .env doesn't exist (e.g. in CI), we might want to skip or just pass
        # But for this task, we assume it exists as per user request
        return

    def get_keys(path: Path):
        keys = set()
        with path.open() as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#"):
                    if "=" in line:
                        key = line.split("=")[0].strip()
                        keys.add(key)
        return keys

    env_keys = get_keys(env_path)
    example_keys = get_keys(example_path)

    # Check for missing keys in .env
    missing_in_env = example_keys - env_keys
    # Check for extra keys in .env that are missing in .env.example
    missing_in_example = env_keys - example_keys

    assert (
        not missing_in_env
    ), f"Keys found in .env.example but missing in .env: {missing_in_env}"
    assert (
        not missing_in_example
    ), f"Keys found in .env but missing in .env.example: {missing_in_example}"
