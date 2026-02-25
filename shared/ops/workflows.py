from datetime import timedelta

from temporalio import activity, workflow
from temporalio.common import RetryPolicy
from shared.models.schemas import BackupParams, BackupResult


@activity.defn
async def run_backup_activity(params: BackupParams) -> BackupResult:
    """
    Activity to perform the actual backup operations.
    """
    from shared.ops.backup import backup_postgres, backup_s3_files

    db_url = params.db_url
    s3_bucket = params.s3_bucket
    source_bucket = params.source_bucket
    backup_bucket = params.backup_bucket

    pg_key = None
    file_count = None

    if db_url and s3_bucket:
        pg_key = backup_postgres(db_url, s3_bucket)

    if source_bucket and backup_bucket:
        file_count = backup_s3_files(source_bucket, backup_bucket)

    return BackupResult(postgres_backup_key=pg_key, s3_files_backed_up=file_count)


@workflow.defn
class BackupWorkflow:
    @workflow.run
    async def run(self, params: BackupParams) -> BackupResult:
        """
        Workflow to orchestrate the backup process.
        """
        return await workflow.execute_activity(
            run_backup_activity,
            params,
            start_to_close_timeout=timedelta(minutes=60),
            retry_policy=RetryPolicy(
                maximum_attempts=3,
                initial_interval=timedelta(seconds=10),
                backoff_coefficient=2.0,
            ),
        )
