from datetime import timedelta
from typing import TYPE_CHECKING

from temporalio import workflow

from shared.type_checking import type_check

if TYPE_CHECKING:
    pass


@type_check
@workflow.defn
class ScriptExecutionWorkflow:
    @workflow.run
    async def run(self, data: dict) -> dict:
        """Workflow to ensure durable execution of scripts on the worker."""
        return await workflow.execute_activity(
            "execute_script_activity",
            data,
            start_to_close_timeout=timedelta(minutes=10),
        )
