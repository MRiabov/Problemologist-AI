from datetime import timedelta
from typing import TYPE_CHECKING

from temporalio import workflow

from shared.type_checking import type_check

# Import activities for type hinting in workflow (optional but good for clarity)
if TYPE_CHECKING:
    pass


@type_check
@workflow.defn
class SimulationWorkflow:
    @workflow.run
    async def run(self, params: dict) -> str:
        compound_json = params["compound_json"]
        episode_id = params["episode_id"]
        # Extract failure flags for integration testing
        simulate_failures = params.get("simulate_failures", {})

        try:
            mjcf_data = await workflow.execute_activity(
                "compile_mjcf_activity",
                args=[compound_json, simulate_failures],
                start_to_close_timeout=timedelta(minutes=1),
            )

            sim_results = await workflow.execute_activity(
                "run_simulation_activity",
                args=[mjcf_data, simulate_failures],
                start_to_close_timeout=timedelta(minutes=5),
            )

            video_path = await workflow.execute_activity(
                "render_video_activity",
                args=[sim_results, simulate_failures],
                start_to_close_timeout=timedelta(minutes=2),
            )

            s3_path = await workflow.execute_activity(
                "upload_to_s3_activity",
                args=[video_path, simulate_failures],
                start_to_close_timeout=timedelta(minutes=2),
            )

            await workflow.execute_activity(
                "update_trace_activity",
                {
                    "episode_id": episode_id,
                    "s3_path": s3_path,
                    "asset_type": "VIDEO",
                    "status": "completed",
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
                    "asset_type": "ERROR",
                    "status": "failed",
                    "error": str(e),
                },
                start_to_close_timeout=timedelta(seconds=30),
            )
            # Re-raise so Temporal knows it failed
            raise e
