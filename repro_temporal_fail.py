import asyncio
from unittest.mock import MagicMock
from temporalio.worker import Worker
from controller.workflows.execution import ScriptExecutionWorkflow


async def main():
    try:
        client = MagicMock()
        client.config.return_value = {"plugins": []}
        client.identity = "test"
        client.namespace = "default"

        # This should trigger validation
        worker = Worker(
            client,
            task_queue="test-queue",
            workflows=[ScriptExecutionWorkflow],
            activities=[],
        )
        print("Workflow validation successful!")
    except Exception as e:
        print(f"Workflow validation failed: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(main())
