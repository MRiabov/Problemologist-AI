from pathlib import Path
import sqlite3
from langgraph.checkpoint.sqlite import SqliteSaver


def get_checkpointer(db_path: str = ".agent_storage/checkpoints.sqlite") -> SqliteSaver:
    """
    Get a pre-configured SqliteSaver for LangGraph.
    Ensures the database directory exists.
    """
    db_file = Path(db_path)
    db_file.parent.mkdir(parents=True, exist_ok=True)

    # Create connection with check_same_thread=False for async/threaded contexts
    conn = sqlite3.connect(str(db_file), check_same_thread=False)

    # Initialize the saver with the connection
    checkpointer = SqliteSaver(conn)

    return checkpointer
