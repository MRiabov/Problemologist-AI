import os
import boto3
from pathlib import Path
import structlog
from botocore.exceptions import ClientError

logger = structlog.get_logger(__name__)

class StorageClient:
    """Simple S3 client for uploading simulation artifacts."""

    def __init__(self):
        self.endpoint = os.getenv("S3_ENDPOINT")
        self.access_key = os.getenv("S3_ACCESS_KEY")
        self.secret_key = os.getenv("S3_SECRET_KEY")
        self.bucket = os.getenv("S3_BUCKET", "simulation-artifacts")
        self.region = os.getenv("S3_REGION", "us-east-1")

        self.client = boto3.client(
            "s3",
            endpoint_url=self.endpoint,
            aws_access_key_id=self.access_key,
            aws_secret_access_key=self.secret_key,
            region_name=self.region,
        )

    def upload_file(self, file_path: Path, object_name: str) -> str:
        """Uploads a file to S3 and returns a public/presigned URL."""
        try:
            self.client.upload_file(str(file_path), self.bucket, object_name)
            
            # Generate a presigned URL (valid for 7 days)
            url = self.client.generate_presigned_url(
                'get_object',
                Params={'Bucket': self.bucket, 'Key': object_name},
                ExpiresIn=604800
            )
            logger.info("file_uploaded_to_s3", bucket=self.bucket, key=object_name, url=url)
            return url
        except ClientError as e:
            logger.error("s3_upload_failed", error=str(e))
            raise
