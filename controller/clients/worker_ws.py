import asyncio
from urllib.parse import urlparse, urlunparse

from websockets.asyncio.client import connect as websocket_connect

from shared.workers.schema import WorkerLightRpcRequest, WorkerLightRpcResponse


def _http_to_ws_url(base_url: str, path: str = "/ws") -> str:
    parsed = urlparse(base_url.rstrip("/"))
    scheme = "wss" if parsed.scheme == "https" else "ws"
    return urlunparse(
        (
            scheme,
            parsed.netloc,
            parsed.path.rstrip("/") + path,
            parsed.params,
            parsed.query,
            parsed.fragment,
        )
    )


class WorkerLightWebSocketClient:
    """Single-connection RPC client for worker-light websocket transport."""

    def __init__(self, base_url: str, headers: dict[str, str]):
        self.ws_url = _http_to_ws_url(base_url)
        self.headers = headers
        self._connection = None
        self._lock = asyncio.Lock()

    async def _ensure_connection(self):
        if self._connection is not None and not getattr(
            self._connection, "closed", False
        ):
            return self._connection

        self._connection = await websocket_connect(
            self.ws_url,
            additional_headers=self.headers,
            open_timeout=10.0,
            close_timeout=5.0,
            ping_interval=20.0,
            ping_timeout=20.0,
            max_size=None,
        )
        return self._connection

    async def request(self, request: WorkerLightRpcRequest) -> WorkerLightRpcResponse:
        async with self._lock:
            connection = await self._ensure_connection()
            try:
                await connection.send(request.model_dump_json())
                raw_message = await connection.recv()
                if isinstance(raw_message, bytes):
                    raw_message = raw_message.decode("utf-8")
                response = WorkerLightRpcResponse.model_validate_json(raw_message)
                if response.request_id != request.request_id:
                    raise RuntimeError(
                        "worker-light websocket response correlation mismatch"
                    )
                return response
            except Exception:
                await self.aclose()
                raise

    async def aclose(self) -> None:
        if self._connection is None:
            return

        try:
            await self._connection.close()
        except Exception:
            # Teardown must not turn a successful RPC into a transport failure.
            # Some worker-light runs do not return a clean close frame.
            pass
        finally:
            self._connection = None
