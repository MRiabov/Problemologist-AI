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
    async def run(self, compound_json: str) -> str:
        mjcf_data = await workflow.execute_activity(
            "compile_mjcf_activity",
            compound_json,
            start_to_close_timeout=timedelta(minutes=1),
        )

        sim_results = await workflow.execute_activity(
            "run_simulation_activity",
            mjcf_data,
            start_to_close_timeout=timedelta(minutes=5),
        )

        video_path = await workflow.execute_activity(
            "render_video_activity",
            sim_results,
            start_to_close_timeout=timedelta(minutes=2),
        )

        s3_url = await workflow.execute_activity(
            "upload_to_s3_activity",
            video_path,
            start_to_close_timeout=timedelta(minutes=2),
        )

        await workflow.execute_activity(
            "update_trace_activity",
            s3_url,
            start_to_close_timeout=timedelta(seconds=30),
        )

        return s3_url
