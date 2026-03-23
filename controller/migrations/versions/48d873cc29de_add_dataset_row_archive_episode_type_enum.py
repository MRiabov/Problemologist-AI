"""add dataset_row_archives episode_type enum

Revision ID: 48d873cc29de
Revises: 4def9501dde7
Create Date: 2026-03-23 18:55:00.000000

"""

from collections.abc import Sequence

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "48d873cc29de"
down_revision: str | Sequence[str] | None = "4def9501dde7"
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
    op.execute(
        """
        ALTER TABLE dataset_row_archives
        ALTER COLUMN episode_type TYPE episodetype
        USING episode_type::text::episodetype
        """
    )


def downgrade() -> None:
    """Downgrade schema."""
    op.execute(
        """
        ALTER TABLE dataset_row_archives
        ALTER COLUMN episode_type TYPE VARCHAR
        USING episode_type::text
        """
    )
    op.execute("DROP TYPE IF EXISTS episodetype")
