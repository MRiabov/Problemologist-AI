from datetime import UTC, datetime

from sqlalchemy import (
    JSON,
    Column,
    DateTime,
    Float,
    Integer,
    String,
    Text,
)
from sqlalchemy.orm import declarative_base

Base = declarative_base()


class CatalogMetadataORM(Base):
    __tablename__ = "catalog_metadata"

    id = Column(Integer, primary_key=True)
    catalog_version = Column(String, nullable=False)
    bd_warehouse_commit = Column(String, nullable=False)
    generated_at = Column(DateTime, default=lambda: datetime.now(UTC))


class COTSItemORM(Base):
    __tablename__ = "parts"

    part_id = Column(String, primary_key=True)
    name = Column(String, index=True, nullable=False)
    category = Column(String, index=True, nullable=False)
    unit_cost = Column(Float, nullable=False)
    weight_g = Column(Float, nullable=False)
    import_recipe = Column(Text, nullable=False)
    metadata_dict = Column("metadata", JSON, nullable=False)
