from langchain_openai import ChatOpenAI
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_anthropic import ChatAnthropic
from langchain_core.language_models.chat_models import BaseChatModel
import os


def get_model(model_name: str, temperature: float = 0.0) -> BaseChatModel:
    """
    Factory function to get a ChatModel instance based on the model name.
    Supports GPT-4o, Gemini 1.5 Pro, and Claude 3.5 Sonnet.

    Args:
        model_name: The name of the model to use (e.g., 'gpt-4o', 'gemini-1.5-pro', 'claude-3-5-sonnet-20240620').
        temperature: The temperature for generation.

    Returns:
        An instance of BaseChatModel.
    """
    if "gpt-4o" in model_name.lower():
        return ChatOpenAI(model=model_name, temperature=temperature)
    elif "gemini" in model_name.lower():
        # Using the newer Gemini integration
        return ChatGoogleGenerativeAI(model=model_name, temperature=temperature)
    elif "claude" in model_name.lower():
        return ChatAnthropic(model=model_name, temperature=temperature)
    else:
        # Fallback to OpenAI if model name is ambiguous but might be an OpenAI model
        try:
            return ChatOpenAI(model=model_name, temperature=temperature)
        except Exception:
            raise ValueError(f"Unsupported model: {model_name}")
