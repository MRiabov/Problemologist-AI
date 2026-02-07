import os
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from shared.observability.persistence import get_db_url, setup_persistence


def test_get_db_url_standard():
    """Test standard PostgreSQL URL."""
    with patch.dict(os.environ, {"DATABASE_URL": "postgresql://user:pass@localhost/db"}):
        assert get_db_url() == "postgresql://user:pass@localhost/db"


def test_get_db_url_asyncpg_conversion():
    """Test conversion of asyncpg URL to standard PostgreSQL URL."""
    with patch.dict(os.environ, {"DATABASE_URL": "postgresql+asyncpg://user:pass@localhost/db"}):
        assert get_db_url() == "postgresql://user:pass@localhost/db"


def test_get_db_url_psycopg_conversion():
    """Test conversion of psycopg URL to standard PostgreSQL URL."""
    with patch.dict(os.environ, {"DATABASE_URL": "postgresql+psycopg://user:pass@localhost/db"}):
        assert get_db_url() == "postgresql://user:pass@localhost/db"


def test_get_db_url_missing():
    """Test that it raises ValueError when DATABASE_URL is missing."""
    with patch.dict(os.environ, {}, clear=True):
        with pytest.raises(ValueError, match="DATABASE_URL environment variable is not set"):
            get_db_url()


@pytest.mark.asyncio
async def test_setup_persistence():
    """Test that setup_persistence calls setup on the saver."""
    with patch("shared.observability.persistence.AsyncConnectionPool") as mock_pool_cls:
        with patch("shared.observability.persistence.PostgresSaver") as mock_saver_cls:
            # Mock the async context manager of the pool
            mock_pool = MagicMock()
            mock_pool_cls.return_value.__aenter__.return_value = mock_pool
            
            # Mock the saver and its setup method
            mock_saver = MagicMock()
            mock_saver.setup = AsyncMock()
            mock_saver_cls.return_value = mock_saver
            
            with patch.dict(os.environ, {"DATABASE_URL": "postgresql://localhost/db"}):
                await setup_persistence()
                
                # Verify pool was created with correct URL
                mock_pool_cls.assert_called_once()
                assert mock_pool_cls.call_args[0][0] == "postgresql://localhost/db"
                
                # Verify setup was called
                mock_saver.setup.assert_called_once()
