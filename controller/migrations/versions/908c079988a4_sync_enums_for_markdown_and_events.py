"""sync enums for markdown and events

Revision ID: 908c079988a4
Revises: 03eb8b34be68
Create Date: 2026-02-26 21:51:00.000000

"""

from collections.abc import Sequence

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "908c079988a4"
down_revision: str | None = "03eb8b34be68"
branch_labels: str | Sequence[str] | None = None
depends_on: str | Sequence[str] | None = None


def upgrade() -> None:
    # Add new values to assettype enum
    op.execute("ALTER TYPE assettype ADD VALUE IF NOT EXISTS 'MARKDOWN'")
    op.execute("ALTER TYPE assettype ADD VALUE IF NOT EXISTS 'LOG'")
    op.execute("ALTER TYPE assettype ADD VALUE IF NOT EXISTS 'ERROR'")
    op.execute("ALTER TYPE assettype ADD VALUE IF NOT EXISTS 'CIRCUIT_DATA'")
    op.execute("ALTER TYPE assettype ADD VALUE IF NOT EXISTS 'TIMELINE'")

    # EVENT is already in 03eb8b34be68, so we don't need to add it here


def downgrade() -> None:
    # Enums are hard to downgrade in Postgres, usually not needed for additions
    pass
