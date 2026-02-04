import uuid
from collections.abc import Generator
from contextlib import contextmanager
from datetime import UTC, datetime
from typing import Any

from sqlalchemy import (
    JSON,
    DateTime,
    Float,
    ForeignKey,
    Integer,
    String,
    Text,
    create_engine,
)
from sqlalchemy.orm import (
    DeclarativeBase,
    Mapped,
    Session,
    mapped_column,
    relationship,
    sessionmaker,
)

from src.workbenches.models import CostBreakdown, ValidationReport


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
    )  # thought, tool, user, handoff
    agent_role: Mapped[str | None] = mapped_column(String(100), nullable=True)
    content: Mapped[str | None] = mapped_column(Text, nullable=True)
    tool_name: Mapped[str | None] = mapped_column(String(255), nullable=True)
    tool_input: Mapped[str | None] = mapped_column(Text, nullable=True)
    tool_output: Mapped[str | None] = mapped_column(Text, nullable=True)
    duration_ms: Mapped[int | None] = mapped_column(Integer, nullable=True)
    metadata_json: Mapped[dict[str, Any] | None] = mapped_column(JSON, nullable=True)

    episode: Mapped["Episode"] = relationship(back_populates="steps")
    artifacts: Mapped[list["Artifact"]] = relationship(
        back_populates="step", cascade="all, delete-orphan"
    )


class ValidationRecord(Base):
    __tablename__ = "validation_records"

    id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
    step_id: Mapped[uuid.UUID] = mapped_column(ForeignKey("steps.id"))
    status: Mapped[str] = mapped_column(String(50))
    manufacturability_score: Mapped[float] = mapped_column(Float)
    violations_json: Mapped[dict[str, Any]] = mapped_column(JSON)
    stl_path: Mapped[str | None] = mapped_column(String(1024), nullable=True)
    error: Mapped[str | None] = mapped_column(Text, nullable=True)

    step: Mapped["Step"] = relationship()
    cost_breakdown: Mapped["CostBreakdownRecord"] = relationship(
        back_populates="validation_record", uselist=False, cascade="all, delete-orphan"
    )


class CostBreakdownRecord(Base):
    __tablename__ = "cost_breakdowns"

    id: Mapped[uuid.UUID] = mapped_column(primary_key=True, default=uuid.uuid4)
    validation_record_id: Mapped[uuid.UUID | None] = mapped_column(
        ForeignKey("validation_records.id"), nullable=True
    )
    process: Mapped[str] = mapped_column(String(50))
    total_cost: Mapped[float] = mapped_column(Float)
    unit_cost: Mapped[float] = mapped_column(Float)
    material_cost_per_unit: Mapped[float] = mapped_column(Float)
    setup_cost: Mapped[float] = mapped_column(Float, default=0.0)
    is_reused: Mapped[bool] = mapped_column(default=False)
    details_json: Mapped[dict[str, Any]] = mapped_column(JSON)
    pricing_explanation: Mapped[str | None] = mapped_column(Text, nullable=True)

    validation_record: Mapped["ValidationRecord"] = relationship(
        back_populates="cost_breakdown"
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
    episode_id: Mapped[uuid.UUID | None] = mapped_column(
        ForeignKey("episodes.id"), nullable=True
    )


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
            autocommit=False,
            autoflush=False,
            expire_on_commit=False,
            bind=self.engine,
        )

    def create_tables(self):
        Base.metadata.create_all(bind=self.engine)

    def close(self):
        """Disposes the engine connection pool."""
        self.engine.dispose()

    def get_session(self) -> Session:
        return self.SessionLocal()

    @contextmanager
    def session_scope(self) -> Generator[Session, None, None]:
        """Provide a transactional scope around a series of operations."""
        session = self.SessionLocal()
        try:
            yield session
            session.commit()
        except Exception:
            session.rollback()
            raise
        finally:
            session.close()

    def create_episode(self, problem_id: str) -> Episode:
        with self.session_scope() as session:
            episode = Episode(problem_id=problem_id)
            session.add(episode)
            session.flush()
            session.refresh(episode)
            return episode

    def log_step(
        self,
        episode_id: uuid.UUID,
        sequence_index: int,
        tool_name: str | None = None,
        tool_input: str | None = None,
        tool_output: str | None = None,
        duration_ms: int | None = None,
        step_type: str = "tool",
        agent_role: str | None = None,
        content: str | None = None,
        metadata_json: dict[str, Any] | None = None,
    ) -> Step:
        with self.session_scope() as session:
            step = Step(
                episode_id=episode_id,
                sequence_index=sequence_index,
                tool_name=tool_name,
                tool_input=tool_input,
                tool_output=tool_output,
                duration_ms=duration_ms,
                type=step_type,
                agent_role=agent_role,
                content=content,
                metadata_json=metadata_json,
            )
            session.add(step)
            session.flush()
            session.refresh(step)
            return step

    def save_artifact(
        self,
        step_id: uuid.UUID,
        artifact_type: str,
        file_path: str,
        content_hash: str | None = None,
    ) -> Artifact:
        with self.session_scope() as session:
            artifact = Artifact(
                step_id=step_id,
                artifact_type=artifact_type,
                file_path=file_path,
                content_hash=content_hash,
            )
            session.add(artifact)
            session.flush()
            session.refresh(artifact)
            return artifact

    def update_episode_status(
        self, episode_id: uuid.UUID, status: str, result_metrics: dict | None = None
    ):
        with self.session_scope() as session:
            episode = session.get(Episode, episode_id)
            if episode:
                episode.status = status
                if result_metrics:
                    episode.result_metrics = result_metrics

    def get_best_cost(self, scenario_id: str) -> float | None:
        with self.session_scope() as session:
            record = session.get(CostRecord, scenario_id)
            return record.best_unit_cost if record else None

    def update_cost_record(
        self, scenario_id: str, unit_cost: float, episode_id: uuid.UUID | None = None
    ) -> bool:
        """
        Updates the cost record if the new unit_cost is lower than the current best.
        Returns True if updated.
        """
        with self.session_scope() as session:
            record = session.get(CostRecord, scenario_id)
            updated = False
            if not record:
                record = CostRecord(
                    scenario_id=scenario_id,
                    best_unit_cost=unit_cost,
                    episode_id=episode_id,
                )
                session.add(record)
                updated = True
            elif unit_cost < record.best_unit_cost:
                record.best_unit_cost = unit_cost
                record.episode_id = episode_id
                updated = True
            return updated

    def save_validation_report(
        self, step_id: uuid.UUID, report: ValidationReport
    ) -> ValidationRecord:
        """Saves a ValidationReport and its associated CostBreakdown to the database."""
        with self.session_scope() as session:
            record = ValidationRecord(
                step_id=step_id,
                status=report.status,
                manufacturability_score=report.manufacturability_score,
                violations_json=[
                    {"description": v.description, "severity": v.severity}
                    for v in report.violations
                ],
                stl_path=report.stl_path,
                error=report.error,
            )
            session.add(record)
            session.flush()

            cost_data = report.cost_analysis
            cost_record = CostBreakdownRecord(
                validation_record_id=record.id,
                process=cost_data.process,
                total_cost=cost_data.total_cost,
                unit_cost=cost_data.unit_cost,
                material_cost_per_unit=cost_data.material_cost_per_unit,
                setup_cost=cost_data.setup_cost,
                is_reused=cost_data.is_reused,
                details_json=cost_data.details,
                pricing_explanation=cost_data.pricing_explanation,
            )
            session.add(cost_record)
            session.flush()
            session.refresh(record)
            return record
