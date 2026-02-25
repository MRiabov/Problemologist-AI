from datetime import timedelta
from typing import TYPE_CHECKING

from temporalio import workflow

from shared.enums import AssetType, EpisodeStatus
from shared.type_checking import type_check

# Import activities for type hinting in workflow (optional but good for clarity)
if TYPE_CHECKING:
    pass


@type_check
@workflow.defn
class SimulationWorkflow:
    @workflow.run
    async def run(self, params: dict) -> str:
        import json

        compound_json_str = params["compound_json"]
        episode_id = params["episode_id"]
        # Extract failure flags for integration testing
        simulate_failures = params.get("simulate_failures", {})

        # Also check inside compound_json if it's a JSON string
        try:
            cj = json.loads(compound_json_str)
            if cj.get("fail_upload"):
                simulate_failures["s3_upload"] = True
        except:
            pass

        from temporalio.common import RetryPolicy

        retry_policy = RetryPolicy(maximum_attempts=3)

        try:
            mjcf_data = await workflow.execute_activity(
                "compile_mjcf_activity",
                args=[compound_json_str, simulate_failures],
                start_to_close_timeout=timedelta(minutes=1),
                retry_policy=retry_policy,
            )

            sim_results = await workflow.execute_activity(
                "run_simulation_activity",
                args=[mjcf_data, simulate_failures],
                start_to_close_timeout=timedelta(minutes=5),
                retry_policy=retry_policy,
            )

            video_path = await workflow.execute_activity(
                "render_video_activity",
                args=[sim_results, simulate_failures],
                start_to_close_timeout=timedelta(minutes=2),
                retry_policy=retry_policy,
            )

            s3_path = await workflow.execute_activity(
                "upload_to_s3_activity",
                args=[video_path, simulate_failures],
                start_to_close_timeout=timedelta(minutes=2),
                retry_policy=retry_policy,
            )

            await workflow.execute_activity(
                "update_trace_activity",
                {
                    "episode_id": episode_id,
                    "s3_path": s3_path,
                    "asset_type": AssetType.VIDEO,
                    "status": EpisodeStatus.COMPLETED,
                },
                start_to_close_timeout=timedelta(seconds=30),
            )

            return s3_path

        except Exception as e:
            # On any failure, ensure we mark the episode as FAILED
            await workflow.execute_activity(
                "update_trace_activity",
                {
                    "episode_id": episode_id,
                    "s3_path": "",
                    "asset_type": AssetType.ERROR,
                    "status": EpisodeStatus.FAILED,
                    "error": str(e),
                },
                start_to_close_timeout=timedelta(seconds=30),
            )
            # Re-raise so Temporal knows it failed
            raise e
