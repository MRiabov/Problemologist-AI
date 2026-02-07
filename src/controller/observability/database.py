import uuid
from typing import Any, Dict, List, Optional
from langchain_core.callbacks import BaseCallbackHandler
from src.controller.persistence.db import get_sessionmaker
from src.controller.persistence.models import Trace

class DatabaseCallbackHandler(BaseCallbackHandler):
    """Callback handler that stores traces in the database."""
    
    def __init__(self, episode_id: uuid.UUID):
        self.episode_id = episode_id
        self.session_factory = get_sessionmaker()

    async def on_chain_start(
        self, serialized: Dict[str, Any], inputs: Dict[str, Any], **kwargs: Any
    ) -> None:
        pass

    async def on_tool_start(
        self, serialized: Dict[str, Any], input_str: str, **kwargs: Any
    ) -> None:
        async with self.session_factory() as db:
            trace = Trace(
                episode_id=self.episode_id,
                raw_trace={
                    "type": "tool_start",
                    "name": serialized.get("name"),
                    "input": input_str
                }
            )
            db.add(trace)
            await db.commit()

    async def on_tool_end(self, output: str, **kwargs: Any) -> None:
        async with self.session_factory() as db:
            trace = Trace(
                episode_id=self.episode_id,
                raw_trace={
                    "type": "tool_end",
                    "output": output
                }
            )
            db.add(trace)
            await db.commit()

    async def on_llm_new_token(self, token: str, **kwargs: Any) -> None:
        # We might not want to save every token to DB, maybe just full completions
        pass

    async def on_llm_end(self, response: Any, **kwargs: Any) -> None:
        async with self.session_factory() as db:
            # Extract content from the first generation
            content = ""
            if response.generations and response.generations[0]:
                content = response.generations[0][0].text
            
            trace = Trace(
                episode_id=self.episode_id,
                raw_trace={
                    "type": "llm_end",
                    "content": content
                }
            )
            db.add(trace)
            await db.commit()
