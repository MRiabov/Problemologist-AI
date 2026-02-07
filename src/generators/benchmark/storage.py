import io
import os
import zipfile
import uuid
from typing import List, Dict, Any, Optional
from datetime import datetime
import asyncio

import boto3
import structlog
from sqlalchemy.ext.asyncio import AsyncSession
from botocore.exceptions import ClientError

from .models import BenchmarkAsset
from .schema import BenchmarkAssetModel
from src.controller.persistence.db import get_db

logger = structlog.get_logger()

class BenchmarkStorage:
    def __init__(self):
        self.s3 = boto3.client(
            "s3",
            endpoint_url=os.getenv("S3_ENDPOINT_URL"),
            aws_access_key_id=os.getenv("AWS_ACCESS_KEY_ID"),
            aws_secret_access_key=os.getenv("AWS_SECRET_ACCESS_KEY"),
            region_name=os.getenv("AWS_REGION", "us-east-1"),
        )
        self.source_bucket = os.getenv("BENCHMARK_SOURCE_BUCKET", "benchmarks-source")
        self.assets_bucket = os.getenv("BENCHMARK_ASSETS_BUCKET", "benchmarks-assets")

    async def save_asset(
        self,
        benchmark_id: uuid.UUID,
        script: str,
        mjcf: str,
        images: List[bytes],
        metadata: Dict[str, Any],
        db: AsyncSession,
    ) -> BenchmarkAsset:
        """
        Uploads artifacts to S3 and saves record to DB.
        """
        
        # 1. Upload Script
        script_key = f"{benchmark_id}/script.py"
        await self._upload_string(self.source_bucket, script_key, script, "text/x-python")
        script_url = self._get_url(self.source_bucket, script_key)

        # 2. Upload MJCF
        mjcf_key = f"{benchmark_id}/model.xml"
        await self._upload_string(self.assets_bucket, mjcf_key, mjcf, "application/xml")
        mjcf_url = self._get_url(self.assets_bucket, mjcf_key)

        # 3. Zip and Upload Images
        zip_buffer = io.BytesIO()
        with zipfile.ZipFile(zip_buffer, "w", zipfile.ZIP_DEFLATED) as zip_file:
            for i, img_data in enumerate(images):
                zip_file.writestr(f"view_{i}.png", img_data)
        
        zip_buffer.seek(0)
        bundle_key = f"{benchmark_id}/views.zip"
        
        # Use run_in_executor for blocking S3 IO
        await asyncio.to_thread(
            self.s3.upload_fileobj,
            zip_buffer, 
            self.assets_bucket, 
            bundle_key, 
            ExtraArgs={"ContentType": "application/zip"}
        )
        bundle_url = self._get_url(self.assets_bucket, bundle_key)

        # 4. Save to DB
        asset_model = BenchmarkAssetModel(
            benchmark_id=benchmark_id,
            mjcf_url=mjcf_url,
            build123d_url=script_url,
            preview_bundle_url=bundle_url,
            benchmark_metadata=metadata,
            difficulty_score=metadata.get("difficulty_score", 0.0),
            random_variants=[] # TODO: Handle variants logic if needed
        )
        
        db.add(asset_model)
        await db.commit()
        await db.refresh(asset_model)

        return BenchmarkAsset(
            benchmark_id=asset_model.benchmark_id,
            mjcf_url=asset_model.mjcf_url,
            build123d_url=asset_model.build123d_url,
            preview_bundle_url=asset_model.preview_bundle_url,
            random_variants=asset_model.random_variants,
            difficulty_score=asset_model.difficulty_score,
            metadata=asset_model.benchmark_metadata
        )

    async def _upload_string(self, bucket: str, key: str, content: str, content_type: str):
        def _upload():
            self.s3.put_object(
                Bucket=bucket,
                Key=key,
                Body=content.encode("utf-8"),
                ContentType=content_type
            )
        await asyncio.to_thread(_upload)

    def _get_url(self, bucket: str, key: str) -> str:
        # If using MinIO or S3, construct URL. 
        endpoint = os.getenv("S3_ENDPOINT_URL", "https://s3.amazonaws.com")
        if "minio" in endpoint or "localhost" in endpoint:
             return f"{endpoint}/{bucket}/{key}"
        return f"https://{bucket}.s3.amazonaws.com/{key}"
