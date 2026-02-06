from typing import Optional
from pydantic_settings import BaseSettings, SettingsConfigDict


class AgentSettings(BaseSettings):
    """Configuration for the Engineer Agent."""
    
    openai_api_key: str
    anthropic_api_key: Optional[str] = None
    
    # URL for the Spec 001 Worker API
    spec_001_api_url: str = "http://worker:8001"
    
    # DB connection for checkpointing (if using PostgresSaver in future)
    db_connection_string: Optional[str] = None
    
    # Default session/thread ID for the agent
    default_session_id: str = "default-agent-session"

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore"
    )

settings = AgentSettings()
