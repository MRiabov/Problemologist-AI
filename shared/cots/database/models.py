from sqlalchemy import JSON, Column, Float, String, Text
from sqlalchemy.orm import declarative_base

Base = declarative_base()


class COTSItemORM(Base):
    __tablename__ = "parts"

    part_id = Column(String, primary_key=True)
    name = Column(String, index=True, nullable=False)
    category = Column(String, index=True, nullable=False)
    unit_cost = Column(Float, nullable=False)
    weight_g = Column(Float, nullable=False)
    import_recipe = Column(Text, nullable=False)
    metadata_dict = Column("metadata", JSON, nullable=False)
