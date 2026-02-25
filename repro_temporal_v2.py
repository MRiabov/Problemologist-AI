import asyncio
import traceback

print("Importing Temporal internal modules...")
from temporalio.worker import _workflow

print("Importing workflow...")
from controller.workflows.execution import ScriptExecutionWorkflow

print("Importing schemas...")
from shared.workers.schema import ScriptExecutionRequest, ExecuteResponse


async def main():
    print("Testing ScriptExecutionWorkflow validation...")
    try:
        print("Calling _workflow._Definition.from_class...")
        definition = _workflow._Definition.from_class(ScriptExecutionWorkflow)
        print("Workflow validation successful!")
        print(f"Workflow name: {definition.name}")
    except Exception as e:
        print(f"\nWorkflow validation failed: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(main())
