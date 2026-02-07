import subprocess
import gzip
import shutil
import os
from pathlib import Path
import boto3
from datetime import datetime, UTC
import structlog

logger = structlog.get_logger(__name__)

def backup_postgres(db_url: str, s3_bucket: str, s3_key_prefix: str = "backups/postgres"):
    """
    Dumps the postgres database using pg_dump, compresses it, and uploads to S3.
    """
    timestamp = datetime.now(UTC).strftime("%Y%m%d_%H%M%S")
    dump_path = Path(f"db_dump_{timestamp}.sql")
    gz_path = Path(f"{dump_path}.gz")
    
    try:
        logger.info("Starting postgres backup", s3_bucket=s3_bucket)
        
        # Run pg_dump
        # We assume pg_dump is in the PATH and db_url is a valid libpq connection string
        subprocess.run(
            ["pg_dump", "--dbname", db_url, "-f", str(dump_path)],
            check=True,
            capture_output=True,
            text=True
        )
        
        # Compress
        with dump_path.open('rb') as f_in:
            with gzip.open(gz_path, 'wb') as f_out:
                shutil.copyfileobj(f_in, f_out)
        
        # Upload to S3
        s3 = boto3.client('s3')
        s3_key = f"{s3_key_prefix}/{gz_path.name}"
        s3.upload_file(str(gz_path), s3_bucket, s3_key)
        
        logger.info("Postgres backup completed and uploaded", s3_key=s3_key)
        return s3_key
    except Exception as e:
        logger.error("Postgres backup failed", error=str(e))
        raise
    finally:
        # Cleanup temporary files
        if dump_path.exists():
            dump_path.unlink()
        if gz_path.exists():
            gz_path.unlink()

def backup_s3_files(source_bucket: str, backup_bucket: str, backup_prefix: str = "backups/files"):
    """
    Syncs files from source bucket to backup bucket.
    """
    try:
        logger.info("Starting S3 sync", source=source_bucket, dest=backup_bucket)
        s3 = boto3.resource('s3')
        source = s3.Bucket(source_bucket)
        dest = s3.Bucket(backup_bucket)
        
        timestamp = datetime.now(UTC).strftime("%Y-%m-%d")
        
        count = 0
        for obj in source.objects.all():
            copy_source = {
                'Bucket': source_bucket,
                'Key': obj.key
            }
            # Prefix with date to keep snapshots
            dest_key = f"{backup_prefix}/{timestamp}/{obj.key}"
            dest.copy(copy_source, dest_key)
            count += 1
            
        logger.info("S3 sync completed", source=source_bucket, dest=backup_bucket, file_count=count)
        return count
    except Exception as e:
        logger.error("S3 backup failed", error=str(e))
        raise
