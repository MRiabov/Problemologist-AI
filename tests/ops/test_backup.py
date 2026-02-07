import subprocess
from unittest.mock import MagicMock, mock_open, patch

import pytest

from shared.ops.backup import backup_postgres, backup_s3_files


def test_backup_postgres_success():
    # Mock all external dependencies
    with (
        patch("subprocess.run") as mock_run,
        patch("boto3.client") as mock_boto,
        patch("builtins.open", mock_open(read_data=b"dummy data")),
        patch("shared.ops.backup.Path.open", mock_open(read_data=b"dummy data")),
        patch("gzip.open", mock_open()) as mock_gzip,
        patch("shutil.copyfileobj") as mock_copy,
        patch("shared.ops.backup.Path.unlink") as mock_unlink,
        patch("shared.ops.backup.Path.exists", return_value=True),
    ):
        mock_run.return_value = MagicMock(returncode=0)
        mock_s3 = MagicMock()
        mock_boto.return_value = mock_s3

        db_url = "postgresql://user:pass@localhost/db"
        s3_bucket = "my-backup-bucket"

        result = backup_postgres(db_url, s3_bucket)

        # Verify pg_dump call
        mock_run.assert_called_once()
        args = mock_run.call_args[0][0]
        assert "pg_dump" in args
        assert "--dbname" in args
        assert db_url in args

        # Verify S3 upload
        mock_s3.upload_file.assert_called_once()
        upload_call_args = mock_s3.upload_file.call_args[0]
        # upload_file(filename, bucket, key)
        assert upload_call_args[1] == s3_bucket
        assert upload_call_args[2].startswith("backups/postgres/")

        assert result.startswith("backups/postgres/")

        # Verify cleanup
        assert mock_unlink.call_count >= 2


def test_backup_s3_files_success():
    with patch("boto3.resource") as mock_boto:
        mock_s3 = MagicMock()
        mock_boto.return_value = mock_s3

        mock_source = MagicMock()
        mock_dest = MagicMock()

        # side_effect for multiple calls to Bucket()
        mock_s3.Bucket.side_effect = [mock_source, mock_dest]

        mock_obj = MagicMock()
        mock_obj.key = "test-file.txt"
        mock_source.objects.all.return_value = [mock_obj]

        source_bucket = "source-bucket"
        backup_bucket = "backup-bucket"

        count = backup_s3_files(source_bucket, backup_bucket)

        assert count == 1
        mock_dest.copy.assert_called_once()
        copy_args = mock_dest.copy.call_args[0]
        # copy(CopySource, Key, ...)
        assert copy_args[0]["Bucket"] == source_bucket
        assert copy_args[0]["Key"] == "test-file.txt"
        assert copy_args[1].endswith("test-file.txt")


def test_backup_postgres_failure():
    with patch("subprocess.run") as mock_run:
        mock_run.side_effect = subprocess.CalledProcessError(1, "pg_dump")

        with pytest.raises(subprocess.CalledProcessError):
            backup_postgres("url", "bucket")
