"""add_trace_feedback_fields

Revision ID: f189942f2103
Revises: b5f3ce2a7b6d
Create Date: 2026-02-09 19:25:42.617930

"""

from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa
from sqlalchemy.dialects import sqlite

# revision identifiers, used by Alembic.
revision: str = "f189942f2103"
down_revision: Union[str, Sequence[str], None] = "b5f3ce2a7b6d"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    # Traces columns were likely already added in a failed partial migration
    # with op.batch_alter_table("traces", schema=None) as batch_op:
    #     batch_op.add_column(sa.Column("feedback_score", sa.Integer(), nullable=True))
    #     batch_op.add_column(sa.Column("feedback_comment", sa.String(), nullable=True))

    with op.batch_alter_table("benchmark_assets", schema=None) as batch_op:
        batch_op.alter_column(
            "benchmark_id",
            existing_type=sa.NUMERIC(),
            type_=sa.Uuid(),
            existing_nullable=False,
        )

    # Update episodes status enum if needed, but skipping complex enum changes for now to be safe


def downgrade() -> None:
    """Downgrade schema."""
    with op.batch_alter_table("benchmark_assets", schema=None) as batch_op:
        batch_op.alter_column(
            "benchmark_id",
            existing_type=sa.Uuid(),
            type_=sa.NUMERIC(),
            existing_nullable=False,
        )

    with op.batch_alter_table("traces", schema=None) as batch_op:
        batch_op.drop_column("feedback_comment")
        batch_op.drop_column("feedback_score")
