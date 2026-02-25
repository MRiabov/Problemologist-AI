import os

import httpx
import pytest_asyncio

# Constants
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", os.getenv("WORKER_URL", "http://127.0.0.1:18001"))
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", os.getenv("WORKER_URL", "http://127.0.0.1:18001"))
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")


@pytest_asyncio.fixture
async def controller_client():
    """Connect to real controller service."""
    async with httpx.AsyncClient(base_url=CONTROLLER_URL, timeout=300.0) as client:
        yield client


@pytest_asyncio.fixture
async def worker_light_client():
    """Connect to real worker-light service."""
    async with httpx.AsyncClient(base_url=WORKER_LIGHT_URL, timeout=300.0) as client:
        yield client


@pytest_asyncio.fixture
async def worker_heavy_client():
    """Connect to real worker-heavy service."""
    async with httpx.AsyncClient(base_url=WORKER_HEAVY_URL, timeout=300.0) as client:
        yield client
