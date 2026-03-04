"""create seed lineage enum types

Revision ID: a9d3f7c2e4b1
Revises: 9c4f2e7a1b10
Create Date: 2026-03-04 10:12:00.000000

"""

from collections.abc import Sequence

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "a9d3f7c2e4b1"
down_revision: str | Sequence[str] | None = "9c4f2e7a1b10"
branch_labels: str | Sequence[str] | None = None
depends_on: str | Sequence[str] | None = None


def upgrade() -> None:
    """Upgrade schema."""
    op.execute(
        """
        DO $$
        BEGIN
            IF NOT EXISTS (SELECT 1 FROM pg_type WHERE typname = 'seedmatchmethod') THEN
                CREATE TYPE seedmatchmethod AS ENUM (
                    'runtime_explicit',
                    'exact_task',
                    'no_exact_task_match',
                    'ambiguous_exact_task'
                );
            END IF;
        END $$;
        """
    )
    op.execute(
        """
        DO $$
        BEGIN
            IF NOT EXISTS (SELECT 1 FROM pg_type WHERE typname = 'generationkind') THEN
                CREATE TYPE generationkind AS ENUM (
                    'seeded',
                    'derived',
                    'seeded_eval',
                    'integration_test',
                    'cots_search',
                    'skill_agent'
                );
            END IF;
        END $$;
        """
    )

    op.execute(
        """
        ALTER TABLE episodes
        ALTER COLUMN seed_match_method TYPE seedmatchmethod
        USING seed_match_method::seedmatchmethod
        """
    )
    op.execute(
        """
        ALTER TABLE episodes
        ALTER COLUMN generation_kind TYPE generationkind
        USING generation_kind::generationkind
        """
    )


def downgrade() -> None:
    """Downgrade schema."""
    op.execute(
        """
        ALTER TABLE episodes
        ALTER COLUMN seed_match_method TYPE VARCHAR
        USING seed_match_method::text
        """
    )
    op.execute(
        """
        ALTER TABLE episodes
        ALTER COLUMN generation_kind TYPE VARCHAR
        USING generation_kind::text
        """
    )

    op.execute("DROP TYPE IF EXISTS generationkind")
    op.execute("DROP TYPE IF EXISTS seedmatchmethod")
