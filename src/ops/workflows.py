from datetime import timedelta
from temporalio import workflow, activity
from temporalio.common import RetryPolicy

# Import backup logic
# Use workflow.unsafe.imports_passed_through() for workflow-safe imports if needed, 
# but activities can import normally.
from src.ops.backup import backup_postgres, backup_s3_files

@activity.defn
def run_backup_activity(params: dict) -> dict:
    """
    Activity to perform the actual backup operations.
    """
    db_url = params.get("db_url")
    s3_bucket = params.get("s3_bucket")
    source_bucket = params.get("source_bucket")
    backup_bucket = params.get("backup_bucket")
    
    results = {}
    
    if db_url and s3_bucket:
        pg_key = backup_postgres(db_url, s3_bucket)
        results["postgres_backup_key"] = pg_key
        
    if source_bucket and backup_bucket:
        file_count = backup_s3_files(source_bucket, backup_bucket)
        results["s3_files_backed_up"] = file_count
        
    return results

@workflow.defn
class BackupWorkflow:
    @workflow.run
    async def run(self, params: dict) -> dict:
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
