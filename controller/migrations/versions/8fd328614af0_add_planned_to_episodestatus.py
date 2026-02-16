"""add_planned_to_episodestatus

Revision ID: 8fd328614af0
Revises: 6a5fb04a4024
Create Date: 2026-02-14 21:14:00.733049

"""

from collections.abc import Sequence

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "8fd328614af0"
down_revision: str | Sequence[str] | None = "6a5fb04a4024"
branch_labels: str | Sequence[str] | None = None
depends_on: str | Sequence[str] | None = None


def upgrade() -> None:
    """Upgrade schema."""
    # Use autocommit to allow ALTER TYPE inside a transaction block if supported,
    # or just execute it directly. In PG, ADD VALUE cannot run inside a transaction
    # unless it's a new type.
    op.execute("COMMIT")  # Break out of transaction
    op.execute("ALTER TYPE episodestatus ADD VALUE IF NOT EXISTS 'PLANNED'")


def downgrade() -> None:
    """Downgrade schema."""
    # PostgreSQL does not support removing values from an enum easily.
    pass
