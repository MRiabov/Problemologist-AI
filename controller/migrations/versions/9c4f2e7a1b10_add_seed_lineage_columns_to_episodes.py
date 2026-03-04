"""add_seed_lineage_columns_to_episodes

Revision ID: 9c4f2e7a1b10
Revises: 7be6f88205bb
Create Date: 2026-03-04 08:15:00.000000

"""

from collections.abc import Sequence

import sqlalchemy as sa
from alembic import op

# revision identifiers, used by Alembic.
revision: str = "9c4f2e7a1b10"
down_revision: str | Sequence[str] | None = "7be6f88205bb"
branch_labels: str | Sequence[str] | None = None
depends_on: str | Sequence[str] | None = None


def upgrade() -> None:
    """Upgrade schema."""
    with op.batch_alter_table("episodes", schema=None) as batch_op:
        batch_op.add_column(sa.Column("seed_id", sa.String(), nullable=True))
        batch_op.add_column(sa.Column("seed_dataset", sa.String(), nullable=True))
        batch_op.add_column(sa.Column("seed_match_method", sa.String(), nullable=True))
        batch_op.add_column(sa.Column("generation_kind", sa.String(), nullable=True))
        batch_op.add_column(sa.Column("parent_seed_id", sa.String(), nullable=True))
        batch_op.create_index(
            batch_op.f("ix_episodes_seed_id"), ["seed_id"], unique=False
        )


def downgrade() -> None:
    """Downgrade schema."""
    with op.batch_alter_table("episodes", schema=None) as batch_op:
        batch_op.drop_index(batch_op.f("ix_episodes_seed_id"))
        batch_op.drop_column("parent_seed_id")
        batch_op.drop_column("generation_kind")
        batch_op.drop_column("seed_match_method")
        batch_op.drop_column("seed_dataset")
        batch_op.drop_column("seed_id")
