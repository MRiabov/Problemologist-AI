"""align enums to uppercase and add glb

Revision ID: 03eb8b34be68
Revises: c8e4cc2bf89c
Create Date: 2026-02-25 07:41:00.000000

"""

from collections.abc import Sequence

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "03eb8b34be68"
down_revision: str | Sequence[str] | None = "c8e4cc2bf89c"
branch_labels: str | Sequence[str] | None = None
depends_on: str | Sequence[str] | None = None


def upgrade() -> None:
    """Upgrade schema to use uppercase enum values and add GLB."""

    # 1. assettype: Convert to uppercase and add GLB
    op.execute("COMMIT")  # Break out of transaction for ALTER TYPE

    # We drop and recreate because it's easier than renaming all values in PG
    # Since this is a dev/test env, we assume no critical data loss on migration
    op.execute("ALTER TABLE assets ALTER COLUMN asset_type TYPE VARCHAR")
    op.execute("DROP TYPE assettype")
    op.execute(
        "CREATE TYPE assettype AS ENUM ('VIDEO', 'MJCF', 'IMAGE', 'STEP', 'STL', 'GLB', 'PYTHON', 'OTHER')"
    )
    op.execute("UPDATE assets SET asset_type = UPPER(asset_type)")
    op.execute(
        "ALTER TABLE assets ALTER COLUMN asset_type TYPE assettype USING asset_type::assettype"
    )

    # 2. episodestatus: Convert to uppercase
    op.execute("ALTER TABLE episodes ALTER COLUMN status TYPE VARCHAR")
    op.execute("DROP TYPE episodestatus")
    op.execute(
        "CREATE TYPE episodestatus AS ENUM ('RUNNING', 'PLANNED', 'COMPLETED', 'FAILED', 'CANCELLED')"
    )
    op.execute("UPDATE episodes SET status = UPPER(status)")
    op.execute(
        "ALTER TABLE episodes ALTER COLUMN status TYPE episodestatus USING status::episodestatus"
    )

    # 3. tracetype: Convert to uppercase
    op.execute("ALTER TABLE traces ALTER COLUMN trace_type TYPE VARCHAR")
    op.execute("DROP TYPE tracetype")
    op.execute(
        "CREATE TYPE tracetype AS ENUM ('TOOL_START', 'TOOL_END', 'LLM_START', 'LLM_END', 'LOG', 'ERROR', 'EVENT')"
    )
    op.execute("UPDATE traces SET trace_type = UPPER(trace_type)")
    op.execute(
        "ALTER TABLE traces ALTER COLUMN trace_type TYPE tracetype USING trace_type::tracetype"
    )


def downgrade() -> None:
    """Downgrade schema."""
    # PostgreSQL does not support removing values from an enum easily.
    pass
