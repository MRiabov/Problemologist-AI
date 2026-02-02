import uuid
from datetime import UTC, datetime
from typing import Any

from sqlalchemy import JSON, DateTime, ForeignKey, Integer, String, Text, create_engine
from sqlalchemy.orm import (
    DeclarativeBase,
    Mapped,
    Session,
    mapped_column,
    relationship,
    sessionmaker,
)


class Base(DeclarativeBase):
    pass


class Episode(Base):
    __tablename__ = "episodes"

    id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
    problem_id: Mapped[str] = mapped_column(String(255))
    start_time: Mapped[datetime] = mapped_column(
        DateTime, default=lambda: datetime.now(UTC)
    )
    status: Mapped[str] = mapped_column(
        String(50), default="started"
    )  # started, success, failure
    result_metrics: Mapped[dict[str, Any] | None] = mapped_column(JSON, nullable=True)

    steps: Mapped[list["Step"]] = relationship(
        back_populates="episode", cascade="all, delete-orphan"
    )


class Step(Base):
    __tablename__ = "steps"

    id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
    episode_id: Mapped[uuid.UUID] = mapped_column(ForeignKey("episodes.id"))
    sequence_index: Mapped[int] = mapped_column(Integer)
    type: Mapped[str] = mapped_column(
        String(50), default="thought"
    )  # thought, tool, user
    tool_name: Mapped[str] = mapped_column(String(255))
    tool_input: Mapped[str] = mapped_column(Text)
    tool_output: Mapped[str | None] = mapped_column(Text, nullable=True)
    duration_ms: Mapped[int | None] = mapped_column(Integer, nullable=True)

    episode: Mapped["Episode"] = relationship(back_populates="steps")
    artifacts: Mapped[list["Artifact"]] = relationship(
        back_populates="step", cascade="all, delete-orphan"
    )


class Artifact(Base):
    __tablename__ = "artifacts"

    id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
    step_id: Mapped[uuid.UUID] = mapped_column(ForeignKey("steps.id"))
    artifact_type: Mapped[str] = mapped_column(String(50))  # mesh, image, log, etc.
    file_path: Mapped[str] = mapped_column(String(1024))
    content_hash: Mapped[str | None] = mapped_column(String(64), nullable=True)

    step: Mapped["Step"] = relationship(back_populates="artifacts")


class CostRecord(Base):
    __tablename__ = "cost_records"

    scenario_id: Mapped[str] = mapped_column(String(255), primary_key=True)
    best_unit_cost: Mapped[float] = mapped_column(nullable=False)
    updated_at: Mapped[datetime] = mapped_column(
        DateTime, default=lambda: datetime.now(UTC), onupdate=lambda: datetime.now(UTC)
    )
    episode_id: Mapped[uuid.UUID | None] = mapped_column(ForeignKey("episodes.id"), nullable=True)


class DatabaseManager:
    def __init__(self, db_url: str = "sqlite:///history.db"):
        self.engine = create_engine(db_url)
        # Enable WAL mode for SQLite
        if db_url.startswith("sqlite"):
            from sqlalchemy import event

            @event.listens_for(self.engine, "connect")
            def set_sqlite_pragma(dbapi_connection, _connection_record):
                cursor = dbapi_connection.cursor()
                cursor.execute("PRAGMA journal_mode=WAL")
                cursor.execute("PRAGMA synchronous=NORMAL")
                cursor.close()

        self.SessionLocal = sessionmaker(
            autocommit=False, autoflush=False, bind=self.engine
        )

    def create_tables(self):
        Base.metadata.create_all(bind=self.engine)

    def get_session(self) -> Session:
        return self.SessionLocal()

    def create_episode(self, problem_id: str) -> Episode:
        session = self.get_session()
        episode = Episode(problem_id=problem_id)
        session.add(episode)
        session.commit()
        session.refresh(episode)
        session.close()
        return episode

    def log_step(
        self,
        episode_id: uuid.UUID,
        sequence_index: int,
        tool_name: str,
        tool_input: str,
        tool_output: str | None = None,
        duration_ms: int | None = None,
        type: str = "tool",
    ) -> Step:
        session = self.get_session()
        step = Step(
            episode_id=episode_id,
            sequence_index=sequence_index,
            tool_name=tool_name,
            tool_input=tool_input,
            tool_output=tool_output,
            duration_ms=duration_ms,
            type=type,
        )
        session.add(step)
        session.commit()
        session.refresh(step)
        session.close()
        return step

    def save_artifact(
        self,
        step_id: uuid.UUID,
        artifact_type: str,
        file_path: str,
        content_hash: str | None = None,
    ) -> Artifact:
        session = self.get_session()
        artifact = Artifact(
            step_id=step_id,
            artifact_type=artifact_type,
            file_path=file_path,
            content_hash=content_hash,
        )
        session.add(artifact)
        session.commit()
        session.refresh(artifact)
        session.close()
        return artifact

    def update_episode_status(
        self, episode_id: uuid.UUID, status: str, result_metrics: dict | None = None
    ):
        session = self.get_session()
        episode = session.get(Episode, episode_id)
        if episode:
            episode.status = status
            if result_metrics:
                episode.result_metrics = result_metrics
            session.commit()
        session.close()

    def get_best_cost(self, scenario_id: str) -> float | None:
        session = self.get_session()
        record = session.get(CostRecord, scenario_id)
        cost = record.best_unit_cost if record else None
        session.close()
        return cost

    def update_cost_record(self, scenario_id: str, unit_cost: float, episode_id: uuid.UUID | None = None) -> bool:
        """Updates the cost record if the new unit_cost is lower than the current best. Returns True if updated."""
        session = self.get_session()
        record = session.get(CostRecord, scenario_id)
        updated = False
        if not record:
            record = CostRecord(scenario_id=scenario_id, best_unit_cost=unit_cost, episode_id=episode_id)
            session.add(record)
            updated = True
        elif unit_cost < record.best_unit_cost:
            record.best_unit_cost = unit_cost
            record.episode_id = episode_id
            updated = True
        
        if updated:
            session.commit()
        session.close()
        return updated
