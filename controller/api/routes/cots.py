from typing import Any

from fastapi import APIRouter, Depends, Query
from sqlalchemy import create_engine, select
from sqlalchemy.orm import Session, sessionmaker

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
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


@router.get("/search")
def search_cots(
    q: str = Query(..., min_length=1, description="Search query"),
    limit: int = Query(20, ge=1, le=100),
    db: Session = Depends(get_db),
) -> list[dict[str, Any]]:
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
            {
                "part_id": item.part_id,
                "name": item.name,
                "category": item.category,
                # Placeholder values as per schema requirements in tests
                "manufacturer": item.metadata_dict.get("manufacturer", "Generic"),
                "price": item.unit_cost,
                "source": "internal",
                "weight_g": item.weight_g,
                "metadata": item.metadata_dict,
            }
        )

    return response


@router.get("/metadata")
def get_catalog_metadata(db: Session = Depends(get_db)) -> dict[str, Any]:
    """
    Get the catalog metadata (version, commit, etc.).
    """
    stmt = select(CatalogMetadataORM).order_by(CatalogMetadataORM.id.desc()).limit(1)
    result = db.scalar(stmt)

    if not result:
        return {
            "catalog_version": "unknown",
            "bd_warehouse_commit": "unknown",
            "generated_at": None,
        }

    return {
        "catalog_version": result.catalog_version,
        "bd_warehouse_commit": result.bd_warehouse_commit,
        "generated_at": result.generated_at.isoformat()
        if result.generated_at
        else None,
    }
