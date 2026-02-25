import sys
import asyncio
from datetime import timedelta

print("v3: Importing temporalio...")
from temporalio.worker import _workflow

print("v3: Importing workflow and schemas...")
from controller.workflows.execution import ScriptExecutionWorkflow
from shared.workers.schema import ScriptExecutionRequest, ExecuteResponse


async def main():
    print("v3: Validating ScriptExecutionWorkflow...")
    try:
        # Direct validation call used by Worker
        _workflow._Definition.from_class(ScriptExecutionWorkflow)
        print("v3: SUCCESS: Workflow is valid!")
        sys.exit(0)
    except Exception as e:
        print(f"v3: FAILURE: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
