"""S3 connection and configuration for the Worker filesystem.

Uses s3fs to connect to MinIO (S3-compatible storage) in development.
Configuration is loaded from environment variables.
"""

import s3fs
import structlog
from pydantic_settings import BaseSettings

logger = structlog.get_logger(__name__)


class S3Config(BaseSettings):
    """S3 connection configuration loaded from environment variables."""

    s3_endpoint: str = "http://localhost:19000"
    s3_access_key: str = "minioadmin"
    s3_secret_key: str = "minioadmin"
    s3_bucket: str = "problemologist"

    model_config = {"env_prefix": "", "case_sensitive": False}


def get_s3_filesystem(config: S3Config | None = None) -> s3fs.S3FileSystem:
    """Create and return an S3FileSystem instance.

    Args:
        config: Optional S3Config. If None, loads from environment.

    Returns:
        Configured s3fs.S3FileSystem instance.
    """
    if config is None:
        config = S3Config()

    # s3fs expects endpoint_url without the scheme prefix
    endpoint_url = config.s3_endpoint

    return s3fs.S3FileSystem(
        key=config.s3_access_key,
        secret=config.s3_secret_key,
        endpoint_url=endpoint_url,
        client_kwargs={"region_name": "us-east-1"},
    )


def verify_s3_connection(fs: s3fs.S3FileSystem, bucket: str) -> bool:
    """Verify that the S3 connection is working.

    Args:
        fs: S3FileSystem instance to verify.
        bucket: Bucket name to check.

    Returns:
        True if connection is successful, False otherwise.
    """
    try:
        # Try to list the bucket to verify connection
        fs.ls(bucket)
        logger.info("s3_connection_verified", bucket=bucket)
        return True
    except Exception as e:
        logger.error("s3_connection_failed", bucket=bucket, error=str(e))
        return False


def get_verified_s3_filesystem(config: S3Config | None = None) -> s3fs.S3FileSystem:
    """Create and verify an S3FileSystem instance.

    Args:
        config: Optional S3Config. If None, loads from environment.

    Returns:
        Verified s3fs.S3FileSystem instance.

    Raises:
        ConnectionError: If S3 connection cannot be established.
    """
    if config is None:
        config = S3Config()

    fs = get_s3_filesystem(config)

    if not verify_s3_connection(fs, config.s3_bucket):
        raise ConnectionError(
            f"Failed to connect to S3 at {config.s3_endpoint}, "
            f"bucket: {config.s3_bucket}"
        )

    return fs
