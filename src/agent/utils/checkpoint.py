from pathlib import Path

from langgraph.checkpoint.sqlite.aio import AsyncSqliteSaver


def get_checkpointer(
    db_path: str = ".agent_storage/checkpoints.sqlite",
) -> AsyncSqliteSaver:
    """
    Get a pre-configured AsyncSqliteSaver for LangGraph.
    Ensures the database directory exists.
    """
    db_file = Path(db_path)
    db_file.parent.mkdir(parents=True, exist_ok=True)

    # Initialize the saver with the path (AsyncSqliteSaver.from_conn_string or constructor)
    # The current LangGraph version uses a .from_conn_string class method or direct init
    checkpointer = AsyncSqliteSaver.from_conn_string(str(db_file))

    return checkpointer
