import uuid
from typing import List, Optional
from sqlalchemy import create_engine, select
from sqlalchemy.orm import Session, sessionmaker, joinedload

# Import models from the environment persistence layer
from src.environment.persistence import Episode, Step, Artifact

from .utils import get_project_root

class DashboardDataLayer:
    def __init__(self, db_path: Optional[str] = None):
        if db_path is None:
            # Default to history.db in the project root
            db_path = str(get_project_root() / "history.db")
        
        db_url = f"sqlite:///{db_path}"
        self.engine = create_engine(db_url)
        
        # Configure SQLite for concurrent read access using WAL mode
        from sqlalchemy import event
        @event.listens_for(self.engine, "connect")
        def set_sqlite_pragma(dbapi_connection, _connection_record):
            cursor = dbapi_connection.cursor()
            cursor.execute("PRAGMA journal_mode=WAL")
            cursor.execute("PRAGMA synchronous=NORMAL")
            cursor.close()

        self.SessionLocal = sessionmaker(bind=self.engine)

    def get_all_episodes(self) -> List[Episode]:
        """Returns all episodes ordered by start time descending."""
        with self.SessionLocal() as session:
            stmt = select(Episode).order_by(Episode.start_time.desc())
            return list(session.scalars(stmt).all())

    def get_episode_by_id(self, episode_id: uuid.UUID | str) -> Optional[Episode]:
        """Returns a single episode with steps and artifacts eagerly loaded."""
        if isinstance(episode_id, str):
            episode_id = uuid.UUID(episode_id)
            
        with self.SessionLocal() as session:
            stmt = (
                select(Episode)
                .where(Episode.id == episode_id)
                .options(
                    joinedload(Episode.steps).joinedload(Step.artifacts)
                )
            )
            # Use unique() because of joinedload with collections
            return session.scalars(stmt).unique().one_or_none()

    def get_step_artifacts(self, step_id: uuid.UUID | str) -> List[Artifact]:
        """Returns all artifacts associated with a specific step."""
        if isinstance(step_id, str):
            step_id = uuid.UUID(step_id)
            
        with self.SessionLocal() as session:
            stmt = select(Artifact).where(Artifact.step_id == step_id)
            return list(session.scalars(stmt).all())

if __name__ == "__main__":
    # Basic validation logic
    try:
        dal = DashboardDataLayer()
        episodes = dal.get_all_episodes()
        print(f"Connection successful. Found {len(episodes)} episodes.")
        for ep in episodes[:5]:
            print(f"Episode: {ep.id} | Problem: {ep.problem_id} | Status: {ep.status}")
    except Exception as e:
        print(f"Data layer validation failed (this is expected if history.db does not exist yet): {e}")
