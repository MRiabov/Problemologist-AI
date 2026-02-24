import httpx
from fastapi import APIRouter, HTTPException, Query, Response
from controller.config.settings import settings
import structlog

logger = structlog.get_logger(__name__)
router = APIRouter(prefix="/api/worker", tags=["worker"])

@router.get("/assets/{path:path}")
async def proxy_worker_asset(
    path: str,
    sessionId: str = Query(..., description="Worker Session ID")
):
    """
    INT-167: Proxy asset requests directly to the worker.
    Only GET is allowed.
    """
    worker_light_url = settings.worker_light_url
    asset_url = f"{worker_light_url}/assets/{path}"

    async with httpx.AsyncClient() as client:
        try:
            resp = await client.get(
                asset_url,
                headers={"X-Session-ID": sessionId},
                timeout=10.0
            )
            if resp.status_code == 404:
                raise HTTPException(status_code=404, detail=f"Asset {path} not found on worker")
            resp.raise_for_status()

            return Response(
                content=resp.content,
                media_type=resp.headers.get("content-type"),
                status_code=resp.status_code,
            )
        except httpx.HTTPStatusError as e:
            raise HTTPException(status_code=e.response.status_code, detail=str(e))
        except Exception as e:
            logger.error("worker_proxy_failed", path=path, error=str(e))
            raise HTTPException(status_code=500, detail=f"Failed to proxy worker asset: {e}")

@router.api_route("/assets/{path:path}", methods=["POST", "PUT", "DELETE", "PATCH"])
async def block_worker_asset_mutations(path: str):
    """
    INT-167: Security contract - block mutations to worker assets via proxy.
    """
    raise HTTPException(status_code=405, detail="Method not allowed for worker assets")
