import os
import uuid
from hashlib import sha256
from pathlib import Path

from sqlalchemy import create_engine, or_
from sqlalchemy.exc import SQLAlchemyError
from sqlalchemy.orm import Session

from shared.observability.events import emit_event
from shared.observability.schemas import COTSSearchEvent

from .database.models import CatalogMetadataORM, COTSItemORM
from .models import COTSItem, SearchQuery

DEFAULT_DB_PATH = os.environ.get("COTS_DB_PATH", "parts.db")


def _catalog_snapshot_id(db_path: str) -> str | None:
    try:
        return sha256(Path(db_path).read_bytes()).hexdigest()
    except Exception:
        return None


def get_catalog_metadata(db_path: str = DEFAULT_DB_PATH) -> dict[str, str | None]:
    """Return the latest catalog metadata snapshot from the COTS database."""
    engine = create_engine(f"sqlite:///{db_path}")

    try:
        with Session(engine) as session:
            meta_stmt = (
                session.query(CatalogMetadataORM)
                .order_by(CatalogMetadataORM.id.desc())
                .limit(1)
            )
            meta_result = meta_stmt.first()
    except SQLAlchemyError:
        meta_result = None

    if not meta_result:
        return {
            "catalog_version": "unknown",
            "bd_warehouse_commit": "unknown",
            "catalog_snapshot_id": _catalog_snapshot_id(db_path),
            "generated_at": "unknown",
        }

    return {
        "catalog_version": meta_result.catalog_version or "unknown",
        "bd_warehouse_commit": meta_result.bd_warehouse_commit or "unknown",
        "catalog_snapshot_id": getattr(meta_result, "catalog_snapshot_id", None)
        or _catalog_snapshot_id(db_path),
        "generated_at": (
            meta_result.generated_at.isoformat() if meta_result.generated_at else "unknown"
        ),
    }


def _cots_item_from_orm(item: COTSItemORM) -> COTSItem:
    return COTSItem(
        part_id=item.part_id,
        name=item.name,
        category=item.category,
        unit_cost=item.unit_cost,
        weight_g=item.weight_g,
        import_recipe=item.import_recipe,
        metadata=item.metadata_dict,
    )


def get_catalog_item_by_part_id(
    part_id: str, db_path: str = DEFAULT_DB_PATH
) -> COTSItem | None:
    """Return an exact COTS catalog item by part_id."""
    engine = create_engine(f"sqlite:///{db_path}")

    try:
        with Session(engine) as session:
            item = session.get(COTSItemORM, part_id)
            if item is None:
                return None
            return _cots_item_from_orm(item)
    except SQLAlchemyError:
        return None


def get_catalog_item_with_metadata(
    part_id: str, db_path: str = DEFAULT_DB_PATH
) -> tuple[COTSItem, dict[str, str | None]] | None:
    """Return an exact COTS catalog item and catalog provenance snapshot."""
    item = get_catalog_item_by_part_id(part_id, db_path=db_path)
    if item is None:
        return None
    return item, get_catalog_metadata(db_path)


def search_parts(query: SearchQuery, db_path: str) -> tuple[list[COTSItem], dict]:
    """
    Search for COTS parts in the database based on a query and constraints.
    Returns (list of items, catalog metadata dict).
    """
    engine = create_engine(f"sqlite:///{db_path}")
    results = []

    with Session(engine) as session:
        stmt = session.query(COTSItemORM)

        # Text query matching name or category
        if query.query:
            search_str = f"%{query.query}%"
            stmt = stmt.filter(
                or_(
                    COTSItemORM.name.ilike(search_str),
                    COTSItemORM.category.ilike(search_str),
                )
            )

        # Apply constraints from the query
        if query.constraints:
            if query.constraints.max_weight_g is not None:
                stmt = stmt.filter(
                    COTSItemORM.weight_g <= float(query.constraints.max_weight_g)
                )
            if query.constraints.max_cost is not None:
                stmt = stmt.filter(
                    COTSItemORM.unit_cost <= float(query.constraints.max_cost)
                )
            if query.constraints.category is not None:
                stmt = stmt.filter(COTSItemORM.category == query.constraints.category)
            if query.constraints.min_size is not None:
                # We fetch more results and filter in-memory due to JSON metadata
                stmt = stmt.limit(query.limit * 5)
            else:
                stmt = stmt.limit(query.limit)
        else:
            stmt = stmt.limit(query.limit)

        orm_items = stmt.all()
        for item in orm_items:
            # Apply in-memory constraints (min_size)
            if query.constraints and query.constraints.min_size is not None:
                min_size = float(query.constraints.min_size)
                volume = item.metadata_dict.get("volume", 0)
                if volume < min_size:
                    continue

            if len(results) >= query.limit:
                break

            results.append(
                COTSItem(
                    part_id=item.part_id,
                    name=item.name,
                    category=item.category,
                    unit_cost=item.unit_cost,
                    weight_g=item.weight_g,
                    import_recipe=item.import_recipe,
                    metadata=item.metadata_dict,
                )
            )

    metadata = get_catalog_metadata(db_path)
    cots_query_id = uuid.uuid4().hex
    metadata["cots_query_id"] = cots_query_id

    # Emit search event
    emit_event(
        COTSSearchEvent(
            query=query.query or str(query.constraints),
            results_count=len(results),
            catalog_version=metadata["catalog_version"],
            bd_warehouse_commit=metadata["bd_warehouse_commit"],
            catalog_snapshot_id=metadata["catalog_snapshot_id"],
            generated_at=metadata["generated_at"],
            cots_query_id=cots_query_id,
            candidates=[p.part_id for p in results],
        )
    )

    return results, metadata
