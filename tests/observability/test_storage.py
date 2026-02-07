import os
import pytest
import boto3
from moto import mock_aws
from shared.observability.storage import S3Client, S3Config

@pytest.fixture
def s3_config():
    return S3Config(
        access_key_id="testing",
        secret_access_key="testing",
        bucket_name="test-bucket",
        region_name="us-east-1",
    )

@pytest.fixture
def s3_client(s3_config):
    with mock_aws():
        conn = boto3.resource("s3", region_name=s3_config.region_name)
        conn.create_bucket(Bucket=s3_config.bucket_name)
        yield S3Client(s3_config)

def test_upload_download(s3_client, tmp_path):
    local_file = tmp_path / "test.txt"
    local_file.write_text("hello world")
    
    key = "test_file.txt"
    
    # Upload
    uploaded_key = s3_client.upload_file(str(local_file), key)
    assert uploaded_key == key
    
    # Download
    download_path = tmp_path / "downloaded.txt"
    s3_client.download_file(key, str(download_path))
    assert download_path.read_text() == "hello world"

@pytest.mark.asyncio
async def test_async_upload_download(s3_client, tmp_path):
    local_file = tmp_path / "async_test.txt"
    local_file.write_text("async hello")
    
    key = "async_test_file.txt"
    
    # Async Upload
    uploaded_key = await s3_client.aupload_file(str(local_file), key)
    assert uploaded_key == key
    
    # Async Download
    download_path = tmp_path / "async_downloaded.txt"
    await s3_client.adownload_file(key, str(download_path))
    assert download_path.read_text() == "async hello"

def test_list_files(s3_client, tmp_path):
    f1 = tmp_path / "f1.txt"
    f1.write_text("c1")
    f2 = tmp_path / "f2.txt"
    f2.write_text("c2")
    
    s3_client.upload_file(str(f1), "file1.txt")
    s3_client.upload_file(str(f2), "folder/file2.txt")
    
    # List all
    files = s3_client.list_files()
    assert len(files) == 2
    keys = [f.key for f in files]
    assert "file1.txt" in keys
    assert "folder/file2.txt" in keys
    
    # List with prefix
    prefix_files = s3_client.list_files(prefix="folder/")
    assert len(prefix_files) == 1
    assert prefix_files[0].key == "folder/file2.txt"

def test_get_presigned_url(s3_client, tmp_path):
    f = tmp_path / "data.txt"
    f.write_text("data")
    key = "test_file.txt"
    s3_client.upload_file(str(f), key)
    
    url = s3_client.get_presigned_url(key)
    assert "test-bucket" in url
    assert key in url
    assert "X-Amz-Algorithm" in url or "Signature=" in url
