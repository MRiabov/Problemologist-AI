from sqlalchemy import create_engine

from .models import Base


def init_db(db_path: str):
    """
    Initialize the database by creating all tables defined in the ORM models.
    """
    engine = create_engine(f"sqlite:///{db_path}")
    Base.metadata.create_all(engine)
