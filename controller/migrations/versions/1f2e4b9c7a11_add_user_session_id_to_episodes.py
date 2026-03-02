"""add_user_session_id_to_episodes

Revision ID: 1f2e4b9c7a11
Revises: 908c079988a4
Create Date: 2026-03-02 10:20:00.000000

"""

from collections.abc import Sequence

import sqlalchemy as sa
from alembic import op

# revision identifiers, used by Alembic.
revision: str = "1f2e4b9c7a11"
down_revision: str | Sequence[str] | None = "908c079988a4"
branch_labels: str | Sequence[str] | None = None
depends_on: str | Sequence[str] | None = None


def upgrade() -> None:
    """Upgrade schema."""
    op.add_column("episodes", sa.Column("user_session_id", sa.UUID(), nullable=True))
    op.create_index(
        op.f("ix_episodes_user_session_id"),
        "episodes",
        ["user_session_id"],
        unique=False,
    )


def downgrade() -> None:
    """Downgrade schema."""
    op.drop_index(op.f("ix_episodes_user_session_id"), table_name="episodes")
    op.drop_column("episodes", "user_session_id")
