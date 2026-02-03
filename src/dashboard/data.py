import uuid
from datetime import datetime
from typing import Any

from sqlalchemy import create_engine, select
from sqlalchemy.orm import joinedload, sessionmaker

# Import models from the environment persistence layer
from src.environment.persistence import Artifact, Episode, Step

from .utils import get_project_root


class DashboardDataLayer:
    def __init__(self, db_path: str | None = None):
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

    def get_all_episodes(self) -> list[Episode]:
        """Returns all episodes ordered by start time descending."""
        with self.SessionLocal() as session:
            stmt = select(Episode).order_by(Episode.start_time.desc())
            return list(session.scalars(stmt).all())

    def get_episode_by_id(self, episode_id: uuid.UUID | str) -> Episode | None:
        """Returns a single episode with steps and artifacts eagerly loaded."""
        if isinstance(episode_id, str):
            try:
                episode_id = uuid.UUID(episode_id)
            except ValueError:
                # Handle mock IDs
                return None

        with self.SessionLocal() as session:
            stmt = (
                select(Episode)
                .where(Episode.id == episode_id)
                .options(joinedload(Episode.steps).joinedload(Step.artifacts))
            )
            # Use unique() because of joinedload with collections
            return session.scalars(stmt).unique().one_or_none()

    def get_step_artifacts(self, step_id: uuid.UUID | str) -> list[Artifact]:
        """Returns all artifacts associated with a specific step."""
        if isinstance(step_id, str):
            step_id = uuid.UUID(step_id)

        with self.SessionLocal() as session:
            stmt = select(Artifact).where(Artifact.step_id == step_id)
            return list(session.scalars(stmt).all())

    def insert_step(self, episode_id: uuid.UUID | str, type: str, content: str):
        """Manually inserts a step into an episode."""
        if isinstance(episode_id, str):
            episode_id = uuid.UUID(episode_id)

        with self.SessionLocal() as session:
            # Get current max sequence index
            stmt = (
                select(Step)
                .where(Step.episode_id == episode_id)
                .order_by(Step.sequence_index.desc())
                .limit(1)
            )
            last_step = session.scalars(stmt).one_or_none()
            next_index = (last_step.sequence_index + 1) if last_step else 0

            new_step = Step(
                episode_id=episode_id,
                sequence_index=next_index,
                type=type,
                tool_input=content,
                tool_name="manual_insert",
            )
            session.add(new_step)
            session.commit()
            return new_step


# Singleton instance
_dal = DashboardDataLayer()


def insert_step(episode_id: str, type: str, content: str):
    """Inserts a manual step into the database."""
    return _dal.insert_step(episode_id, type, content)


def get_all_episodes() -> list[dict[str, Any]]:
    """Returns a list of available episodes from database, falling back to mock if empty."""
    try:
        episodes = _dal.get_all_episodes()
        if episodes:
            return [
                {
                    "id": str(ep.id),
                    "timestamp": ep.start_time,
                    "name": f"Problem: {ep.problem_id[:8]}..."
                    if ep.problem_id
                    else "Untitled Episode",
                }
                for ep in episodes
            ]
    except Exception:
        pass

    return [
        {
            "id": "ep_001",
            "timestamp": datetime(2026, 2, 1, 10, 0, 0),
            "name": "Designing a Cube",
        },
        {
            "id": "ep_002",
            "timestamp": datetime(2026, 2, 1, 11, 30, 0),
            "name": "Complex Linkage Attempt",
        },
    ]


def get_episode_by_id(episode_id: str) -> dict[str, Any]:
    """Returns full episode details including steps from database or mock."""
    try:
        ep = _dal.get_episode_by_id(episode_id)
        if ep:
            return {
                "id": str(ep.id),
                "name": f"Problem: {ep.problem_id}",
                "steps": [
                    {
                        "index": i,
                        "type": step.type,
                        "agent_role": step.agent_role,
                        "content": step.content,
                        "tool_name": step.tool_name,
                        "tool_input": step.tool_input,
                        "tool_output": step.tool_output,
                        "metadata": step.metadata_json,
                        "artifacts": [a.file_path for a in step.artifacts],
                    }
                    for i, step in enumerate(ep.steps)
                ],
            }
    except Exception:
        pass

    if episode_id == "ep_001":
        return {
            "id": "ep_001",
            "name": "Designing a Cube",
            "steps": [
                {
                    "index": 0,
                    "type": "user",
                    "content": "Create a 10mm cube.",
                    "tool_calls": None,
                },
                {
                    "index": 1,
                    "type": "thought",
                    "content": "I need to design a 10mm cube using build123d.",
                    "tool_calls": None,
                },
                {
                    "index": 2,
                    "type": "tool",
                    "content": "Writing the script...",
                    "tool_calls": {
                        "name": "write_file",
                        "inputs": {
                            "path": "design.py",
                            "content": "from build123d import *\nwith BuildPart() as p:\n    Box(10, 10, 10)",  # noqa: E501
                        },
                    },
                    "tool_output": "Successfully wrote design.py",
                    "artifacts": ["design.stl"],
                },
            ],
        }
    return {"id": episode_id, "steps": []}


def get_step_artifacts(episode_id: str, step_index: int) -> list[str]:
    """Returns artifacts for a given step."""
    episode = get_episode_by_id(episode_id)
    if not episode:
        return []
    steps = episode.get("steps", [])
    if step_index < len(steps):
        return steps[step_index].get("artifacts", [])
    return []


def get_latest_episode() -> dict[str, Any] | None:
    """Returns the latest episode."""
    episodes = get_all_episodes()
    if not episodes:
        return None
    # Latest is first in DB (descending), but last in mock list
    # Let's be safe and check timestamp
    sorted_episodes = sorted(episodes, key=lambda x: x["timestamp"], reverse=True)
    return get_episode_by_id(sorted_episodes[0]["id"])
