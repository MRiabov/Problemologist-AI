"""fix assettype enum

Revision ID: c5944eda821c
Revises: c8e4cc2bf89c
Create Date: 2026-02-23 21:05:00.000000

"""
from typing import Sequence, Union

from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision: str = 'c5944eda821c'
down_revision: Union[str, Sequence[str], None] = 'c8e4cc2bf89c'
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # Adding enum values in Postgres
    # Use autocommit block to avoid "ALTER TYPE ... ADD cannot run inside a transaction block"
    with op.get_context().autocommit_block():
        op.execute("ALTER TYPE assettype ADD VALUE IF NOT EXISTS 'glb'")
        op.execute("ALTER TYPE assettype ADD VALUE IF NOT EXISTS 'stl'")
        op.execute("ALTER TYPE assettype ADD VALUE IF NOT EXISTS 'step'")
        # Also keep uppercase just in case, but primary should be lowercase
        op.execute("ALTER TYPE assettype ADD VALUE IF NOT EXISTS 'GLB'")
        op.execute("ALTER TYPE assettype ADD VALUE IF NOT EXISTS 'STL'")
        op.execute("ALTER TYPE assettype ADD VALUE IF NOT EXISTS 'STEP'")


def downgrade() -> None:
    pass
