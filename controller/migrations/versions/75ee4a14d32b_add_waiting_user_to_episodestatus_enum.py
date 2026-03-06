"""Add WAITING_USER to episodestatus enum

Revision ID: 75ee4a14d32b
Revises: a9d3f7c2e4b1
Create Date: 2026-03-05 21:12:31.185007

"""

from typing import Sequence, Union

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "75ee4a14d32b"
down_revision: Union[str, Sequence[str], None] = "a9d3f7c2e4b1"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    op.execute("COMMIT")  # Break out of transaction
    op.execute("ALTER TYPE episodestatus ADD VALUE IF NOT EXISTS 'WAITING_USER'")


def downgrade() -> None:
    """Downgrade schema."""
    # PostgreSQL does not support removing values from an enum easily.
    pass
