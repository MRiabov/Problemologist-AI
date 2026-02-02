from dotenv import load_dotenv
from langchain_core.language_models.chat_models import BaseChatModel
from langchain_openai import ChatOpenAI

from src.agent.utils.config import Config

# Load environment variables
load_dotenv()


def get_model(model_name: str, temperature: float = 0.0) -> BaseChatModel:
    """
    Factory function to get a ChatModel instance based on the model name.
    Uses an OpenAI-compatible provider (configured via environment variables).

    Args:
        model_name: The name of the model to use.
        temperature: The temperature for generation.

    Returns:
        An instance of BaseChatModel.
    """
    return ChatOpenAI(
        model=model_name,
        temperature=temperature,
        openai_api_base=Config.OPENAI_API_BASE,
        openai_api_key=Config.OPENAI_API_KEY,
    )
