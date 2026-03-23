"""add_dataset_row_archives

Revision ID: b0f5e3c1d2a4
Revises: a9d3f7c2e4b1
Create Date: 2026-03-23 00:00:00.000000

"""

from collections.abc import Sequence

import sqlalchemy as sa
from alembic import op

# revision identifiers, used by Alembic.
revision: str = "b0f5e3c1d2a4"
down_revision: str | Sequence[str] | None = "a9d3f7c2e4b1"
branch_labels: str | Sequence[str] | None = None
depends_on: str | Sequence[str] | None = None


def upgrade() -> None:
    """Upgrade schema."""
    op.execute(
        """
        DO $$
        BEGIN
            IF NOT EXISTS (SELECT 1 FROM pg_type WHERE typname = 'episodetype') THEN
                CREATE TYPE episodetype AS ENUM ('benchmark', 'engineer');
            END IF;
        END $$;
        """
    )
    op.create_table(
        "dataset_row_archives",
        sa.Column("id", sa.UUID(), primary_key=True, nullable=False),
        sa.Column(
            "episode_id", sa.UUID(), sa.ForeignKey("episodes.id"), nullable=False
        ),
        sa.Column("user_session_id", sa.UUID(), nullable=True),
        sa.Column("worker_session_id", sa.String(), nullable=True),
        sa.Column("benchmark_id", sa.UUID(), nullable=False),
        sa.Column(
            "episode_type",
            sa.Enum("benchmark", "engineer", name="episodetype"),
            nullable=False,
        ),
        sa.Column("seed_id", sa.String(), nullable=True),
        sa.Column("seed_dataset", sa.String(), nullable=True),
        sa.Column(
            "seed_match_method",
            sa.Enum(
                "runtime_explicit",
                "exact_task",
                "no_exact_task_match",
                "ambiguous_exact_task",
                name="seedmatchmethod",
            ),
            nullable=True,
        ),
        sa.Column(
            "generation_kind",
            sa.Enum(
                "seeded",
                "derived",
                "seeded_eval",
                "integration_test",
                "cots_search",
                "skill_agent",
                name="generationkind",
            ),
            nullable=True,
        ),
        sa.Column("parent_seed_id", sa.String(), nullable=True),
        sa.Column("is_integration_test", sa.Boolean(), nullable=True),
        sa.Column("integration_test_id", sa.String(), nullable=True),
        sa.Column("simulation_run_id", sa.String(), nullable=True),
        sa.Column("cots_query_id", sa.String(), nullable=True),
        sa.Column("review_id", sa.String(), nullable=True),
        sa.Column("revision_hash", sa.String(), nullable=False),
        sa.Column("artifact_hash", sa.String(), nullable=False),
        sa.Column("archive_bucket", sa.String(), nullable=False),
        sa.Column("archive_key", sa.String(), nullable=False),
        sa.Column("archive_sha256", sa.String(), nullable=False),
        sa.Column("archive_size_bytes", sa.Integer(), nullable=False),
        sa.Column("manifest_bucket", sa.String(), nullable=False),
        sa.Column("manifest_key", sa.String(), nullable=False),
        sa.Column("manifest_sha256", sa.String(), nullable=False),
        sa.Column("manifest_size_bytes", sa.Integer(), nullable=False),
        sa.Column(
            "created_at",
            sa.DateTime(),
            nullable=False,
            server_default=sa.text("CURRENT_TIMESTAMP"),
        ),
    )
    op.create_index(
        "ix_dataset_row_archives_episode_id", "dataset_row_archives", ["episode_id"]
    )
    op.create_index(
        "ix_dataset_row_archives_user_session_id",
        "dataset_row_archives",
        ["user_session_id"],
    )
    op.create_index(
        "ix_dataset_row_archives_benchmark_id",
        "dataset_row_archives",
        ["benchmark_id"],
    )


def downgrade() -> None:
    """Downgrade schema."""
    op.drop_index(
        "ix_dataset_row_archives_benchmark_id", table_name="dataset_row_archives"
    )
    op.drop_index(
        "ix_dataset_row_archives_user_session_id", table_name="dataset_row_archives"
    )
    op.drop_index(
        "ix_dataset_row_archives_episode_id", table_name="dataset_row_archives"
    )
    op.drop_table("dataset_row_archives")
    op.execute("DROP TYPE episodetype")
