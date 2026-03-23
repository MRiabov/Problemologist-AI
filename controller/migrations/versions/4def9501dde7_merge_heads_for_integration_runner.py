"""merge heads for integration runner

Revision ID: 4def9501dde7
Revises: 75ee4a14d32b, b0f5e3c1d2a4
Create Date: 2026-03-23 17:38:18.426959

"""

from typing import Sequence, Union

# revision identifiers, used by Alembic.
revision: str = "4def9501dde7"
down_revision: Union[str, Sequence[str], None] = ("75ee4a14d32b", "b0f5e3c1d2a4")
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    """Upgrade schema."""
    pass


def downgrade() -> None:
    """Downgrade schema."""
    pass
