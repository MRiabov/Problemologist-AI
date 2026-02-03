import os

from dotenv import load_dotenv

# Load environment variables from .env file if it exists
load_dotenv()


class Config:
    """
    Agent configuration management, pulling from environment variables.
    """

    # Default model to use
    LLM_MODEL = os.getenv("LLM_MODEL", "gpt-4o")

    # OpenAI-compatible API base URL (e.g., OpenRouter)
    OPENAI_API_BASE = os.getenv("OPENAI_API_BASE", "https://api.openai.com/v1")

    # OpenAI-compatible API key
    OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "")

    # Default temperature for generation
    TEMPERATURE = float(os.getenv("LLM_TEMPERATURE", "0.0"))

    # Work directory for scripts and artifacts
    WORKSPACE_DIR = os.getenv("WORKSPACE_DIR", "./workspace")

    # Safety limits
    MAX_STEPS = int(os.getenv("MAX_STEPS", "100"))

    @classmethod
    def validate(cls):
        """Validates that necessary API keys are present."""
        # Note: We don't necessarily need all of them, just the one for the selected model.
        # LangChain usually errors out if the specific key is missing.
        if not cls.OPENAI_API_KEY:
            raise ValueError("OPENAI_API_KEY is missing. Please set it in your .env file or environment.")
