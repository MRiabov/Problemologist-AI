from fastapi import APIRouter, Depends, Query
from sqlalchemy import create_engine, select
from sqlalchemy.orm import Session, sessionmaker

from controller.api.schemas import CotsMetadataResponse, CotsSearchItem
from shared.cots.database.models import CatalogMetadataORM, COTSItemORM

# Initialize router
router = APIRouter(prefix="/cots", tags=["cots"])

# Database setup
# Using sync engine for SQLite since we don't have aiosqlite easily available.
# FastAPI will run this in a threadpool.
DB_PATH = "parts.db"
engine = create_engine(
    f"sqlite:///{DB_PATH}", connect_args={"check_same_thread": False}, echo=False
)
from shared.cots.database.models import Base

# Ensure tables exist, but handle potential race conditions if multiple workers run this
try:
    Base.metadata.create_all(engine)
except Exception:
    pass

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


@router.get("/search", response_model=list[CotsSearchItem])
def search_cots(
    q: str = Query(..., min_length=1, description="Search query"),
    limit: int = Query(20, ge=1, le=100),
    db: Session = Depends(get_db),
):
    """
    Search for COTS parts by name or category.
    """
    # SQLite LIKE is case-insensitive for ASCII characters by default
    stmt = (
        select(COTSItemORM)
        .where(
            (COTSItemORM.name.like(f"%{q}%")) | (COTSItemORM.category.like(f"%{q}%"))
        )
        .limit(limit)
    )

    results = db.scalars(stmt).all()

    response = []
    for item in results:
        # Map ORM object to response schema expected by tests/clients
        response.append(
            CotsSearchItem(
                part_id=item.part_id,
                name=item.name,
                category=item.category,
                # Placeholder values as per schema requirements in tests
                manufacturer=item.metadata_dict.get("manufacturer", "Generic"),
                price=item.unit_cost,
                source="internal",
                weight_g=item.weight_g,
                metadata_vars=item.metadata_dict,
            )
        )

    return response


@router.get("/metadata", response_model=CotsMetadataResponse)
def get_catalog_metadata(db: Session = Depends(get_db)):
    """
    Get the catalog metadata (version, commit, etc.).
    """
    stmt = select(CatalogMetadataORM).order_by(CatalogMetadataORM.id.desc()).limit(1)
    result = db.scalar(stmt)

    if not result:
        return CotsMetadataResponse(
            catalog_version="unknown",
            bd_warehouse_commit="unknown",
            generated_at=None,
        )

    return CotsMetadataResponse(
        catalog_version=result.catalog_version,
        bd_warehouse_commit=result.bd_warehouse_commit,
        generated_at=result.generated_at.isoformat() if result.generated_at else None,
    )
