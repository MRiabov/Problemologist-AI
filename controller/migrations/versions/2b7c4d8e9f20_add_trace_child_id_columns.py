"""add_trace_child_id_columns

Revision ID: 2b7c4d8e9f20
Revises: 1f2e4b9c7a11
Create Date: 2026-03-02 10:28:00.000000

"""

from collections.abc import Sequence

import sqlalchemy as sa
from alembic import op

# revision identifiers, used by Alembic.
revision: str = "2b7c4d8e9f20"
down_revision: str | Sequence[str] | None = "1f2e4b9c7a11"
branch_labels: str | Sequence[str] | None = None
depends_on: str | Sequence[str] | None = None


def upgrade() -> None:
    """Upgrade schema."""
    op.add_column("traces", sa.Column("simulation_run_id", sa.String(), nullable=True))
    op.add_column("traces", sa.Column("cots_query_id", sa.String(), nullable=True))
    op.add_column("traces", sa.Column("review_id", sa.String(), nullable=True))


def downgrade() -> None:
    """Downgrade schema."""
    op.drop_column("traces", "review_id")
    op.drop_column("traces", "cots_query_id")
    op.drop_column("traces", "simulation_run_id")
