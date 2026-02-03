import logging
import uuid
from datetime import datetime
from pathlib import Path
from typing import List, Optional

from sqlalchemy import select
from sqlalchemy.orm import joinedload

# Import models from the environment persistence layer and shared compiler models
from src.compiler.models import DashEpisode, DashStep, EpisodeSummary
from src.environment.persistence import Artifact, DatabaseManager, Episode, Step

from .utils import get_project_root

logger = logging.getLogger(__name__)


class DashboardDataLayer:
    def __init__(self, db_path: str | None = None):
        if db_path is None:
            # Default to history.db in the project root
            db_path = str(get_project_root() / "history.db")

        db_url = f"sqlite:///{db_path}"
        self.db_manager = DatabaseManager(db_url)

    def get_all_episodes(self) -> List[Episode]:
        """Returns all episodes ordered by start time descending."""
        try:
            with self.db_manager.get_session() as session:
                stmt = select(Episode).order_by(Episode.start_time.desc())
                return list(session.scalars(stmt).all())
        except Exception as e:
            logger.error(f"Error fetching episodes: {e}")
            raise

    def get_episode_by_id(self, episode_id: uuid.UUID | str) -> Optional[Episode]:
        """Returns a single episode with steps and artifacts eagerly loaded."""
        if isinstance(episode_id, str):
            try:
                episode_id = uuid.UUID(episode_id)
            except ValueError:
                # Mock ID or invalid ID
                return None

        try:
            with self.db_manager.get_session() as session:
                stmt = (
                    select(Episode)
                    .where(Episode.id == episode_id)
                    .options(joinedload(Episode.steps).joinedload(Step.artifacts))
                )
                return session.scalars(stmt).unique().one_or_none()
        except Exception as e:
            logger.error(f"Error fetching episode {episode_id}: {e}")
            raise

    def get_step_artifacts(self, step_id: uuid.UUID | str) -> List[Artifact]:
        """Returns all artifacts associated with a specific step."""
        if isinstance(step_id, str):
            step_id = uuid.UUID(step_id)

        try:
            with self.db_manager.get_session() as session:
                stmt = select(Artifact).where(Artifact.step_id == step_id)
                return list(session.scalars(stmt).all())
        except Exception as e:
            logger.error(f"Error fetching artifacts for step {step_id}: {e}")
            raise

    def insert_step(self, episode_id: uuid.UUID | str, type: str, content: str) -> Step:
        """Manually inserts a step into an episode."""
        if isinstance(episode_id, str):
            episode_id = uuid.UUID(episode_id)

        try:
            with self.db_manager.session_scope() as session:
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
                # Session scope commits automatically
                session.flush()
                session.refresh(new_step)
                return new_step
        except Exception as e:
            logger.error(f"Error inserting step: {e}")
            raise


# Global instance (lazily initialized if needed, but here eager for simplicity)
_dal = DashboardDataLayer()


def insert_step(episode_id: str, type: str, content: str):
    """Inserts a manual step into the database."""
    try:
        return _dal.insert_step(episode_id, type, content)
    except Exception as e:
        logger.error(f"Failed to insert step: {e}")
        return None


def get_all_episodes() -> List[EpisodeSummary]:
    """Returns a list of available episodes from database, falling back to mock if empty."""
    try:
        episodes = _dal.get_all_episodes()
        if episodes:
            return [
                EpisodeSummary(
                    id=str(ep.id),
                    timestamp=ep.start_time,
                    name=f"Problem: {ep.problem_id[:8]}..."
                    if ep.problem_id
                    else "Untitled Episode",
                )
                for ep in episodes
            ]
    except Exception as e:
        logger.warning(
            f"Failed to retrieve episodes from DB, using mock data. Error: {e}"
        )

    # Mock fallback
    return [
        EpisodeSummary(
            id="ep_001",
            timestamp=datetime(2026, 2, 1, 10, 0, 0),
            name="Designing a Cube (Mock)",
        ),
        EpisodeSummary(
            id="ep_002",
            timestamp=datetime(2026, 2, 1, 11, 30, 0),
            name="Complex Linkage Attempt (Mock)",
        ),
    ]


def get_episode_by_id(episode_id: str) -> DashEpisode:
    """Returns full episode details including steps from database or mock."""
    try:
        ep = _dal.get_episode_by_id(episode_id)
        if ep:
            steps = [
                DashStep(
                    index=i,
                    type=step.type,
                    agent_role=step.agent_role,
                    content=step.content,
                    tool_name=step.tool_name,
                    tool_input=step.tool_input,
                    tool_output=step.tool_output,
                    metadata=step.metadata_json or {},
                    artifacts=[a.file_path for a in step.artifacts],
                )
                for i, step in enumerate(ep.steps)
            ]
            return DashEpisode(
                id=str(ep.id), name=f"Problem: {ep.problem_id}", steps=steps
            )
    except Exception as e:
        logger.warning(f"Failed to retrieve episode {episode_id} from DB. Error: {e}")

    # Mock Data Fallback
    if episode_id == "ep_001":
        steps = [
            DashStep(
                index=0,
                type="user",
                agent_role=None,
                content="Create a 10mm cube.",
                tool_name=None,
                tool_input=None,
                tool_output=None,
                metadata={},
                artifacts=[],
            ),
            DashStep(
                index=1,
                type="thought",
                agent_role="Planner",
                content="I need to design a 10mm cube using build123d.",
                tool_name=None,
                tool_input=None,
                tool_output=None,
                metadata={},
                artifacts=[],
            ),
            DashStep(
                index=2,
                type="tool",
                agent_role="Draftsman",
                content="Writing the script...",
                tool_name="write_file",
                tool_input="design.py",
                tool_output="Successfully wrote design.py",
                metadata={"path": "design.py"},
                artifacts=["design.stl"],
            ),
        ]
        return DashEpisode(id="ep_001", name="Designing a Cube (Mock)", steps=steps)
    return DashEpisode(id=episode_id, name="Unknown", steps=[])


def get_step_artifacts(episode_id: str, step_index: int) -> List[str]:
    """Returns artifacts for a given step."""
    episode = get_episode_by_id(episode_id)
    if not episode or not episode.steps:
        return []
    if step_index < len(episode.steps):
        return episode.steps[step_index].artifacts
    return []


def get_latest_episode() -> Optional[DashEpisode]:
    """Returns the latest episode."""
    episodes = get_all_episodes()
    if not episodes:
        return None
    sorted_episodes = sorted(episodes, key=lambda x: x.timestamp, reverse=True)
    return get_episode_by_id(sorted_episodes[0].id)
