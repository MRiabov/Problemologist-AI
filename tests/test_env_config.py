from pathlib import Path

import pytest


def test_env_files_consistency():
    """
    Verifies that .env and .env.example have consistent keys.
    This helps ensure that developers update .env.example when adding new config.
    """
    root_dir = Path(__file__).parent.parent
    env_path = root_dir / ".env"
    example_path = root_dir / ".env.example"

    if not env_path.exists():
        pytest.skip(".env file not found, skipping consistency check.")

    def get_keys(path: Path) -> set[str]:
        keys = set()
        if not path.exists():
            return keys
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    key = line.split("=")[0].strip()
                    keys.add(key)
        return keys

    env_keys = get_keys(env_path)
    example_keys = get_keys(example_path)

    # Check for missing keys in .env
    missing_in_env = example_keys - env_keys
    # Check for extra keys in .env that are missing in .env.example
    missing_in_example = env_keys - example_keys

    # Ignore Langfuse and NextAuth internal keys
    ignored_keys = {
        "LANGFUSE_SALT",
        "LANGFUSE_ENCRYPTION_KEY",
        "LANGFUSE_S3_EVENT_UPLOAD_ACCESS_KEY_ID",
        "LANGFUSE_S3_EVENT_UPLOAD_SECRET_ACCESS_KEY",
        "LANGFUSE_S3_EVENT_UPLOAD_FORCE_PATH_STYLE",
        "LANGFUSE_S3_MEDIA_UPLOAD_SECRET_ACCESS_KEY",
        "NEXTAUTH_SECRET",
        "LANGFUSE_BASE_URL",
        "LANGFUSE_S3_EVENT_UPLOAD_ENDPOINT",
        "LANGFUSE_S3_MEDIA_UPLOAD_ACCESS_KEY_ID",
    }
    missing_in_example -= ignored_keys

    assert not missing_in_env, (
        f"Keys found in .env.example but missing in .env: {missing_in_env}"
    )
    assert not missing_in_example, (
        f"Keys found in .env but missing in .env.example: {missing_in_example}"
    )
