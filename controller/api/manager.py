import uuid

from fastapi import WebSocket
import structlog

logger = structlog.get_logger(__name__)


class ConnectionManager:
    """Manages active WebSocket connections for real-time episode updates."""

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance.active_connections: dict[uuid.UUID, list[WebSocket]] = {}
        return cls._instance

    async def connect(self, episode_id: uuid.UUID, websocket: WebSocket):
        """Accept a new connection and register it for an episode."""
        await websocket.accept()
        if episode_id not in self.active_connections:
            self.active_connections[episode_id] = []
        self.active_connections[episode_id].append(websocket)
        logger.info("websocket_connected", episode_id=str(episode_id))

    def disconnect(self, episode_id: uuid.UUID, websocket: WebSocket):
        """Unregister a connection."""
        if episode_id in self.active_connections:
            if websocket in self.active_connections[episode_id]:
                self.active_connections[episode_id].remove(websocket)
            if not self.active_connections[episode_id]:
                del self.active_connections[episode_id]
        logger.info("websocket_disconnected", episode_id=str(episode_id))

    async def broadcast(self, episode_id: uuid.UUID, message: dict):
        """Send a message to all active connections for a given episode."""
        if episode_id not in self.active_connections:
            return

        logger.info(
            "broadcasting_message", episode_id=str(episode_id), type=message.get("type")
        )

        # Collect dead connections to remove them
        dead_connections = []
        for connection in self.active_connections[episode_id]:
            try:
                await connection.send_json(message)
            except Exception as e:
                logger.error(
                    "broadcast_failed", error=str(e), episode_id=str(episode_id)
                )
                dead_connections.append(connection)

        # Cleanup dead connections
        for dead in dead_connections:
            self.disconnect(episode_id, dead)


# Singleton instance
manager = ConnectionManager()
