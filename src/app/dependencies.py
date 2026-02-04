from src.environment.persistence import DatabaseManager
from src.agent.utils.config import Config

def get_db_manager():
    db_path = Config.PROJECT_ROOT / "history.db"
    db_url = f"sqlite:///{db_path}"
    return DatabaseManager(db_url)

def get_db():
    manager = get_db_manager()
    with manager.get_session() as session:
        yield session
