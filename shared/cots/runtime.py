from sqlalchemy import create_engine, or_
from sqlalchemy.orm import Session

from shared.observability.events import emit_event
from shared.observability.schemas import COTSSearchEvent

from .database.models import CatalogMetadataORM, COTSItemORM
from .models import COTSItem, SearchQuery

_global_engine_cache = {}


def search_parts(query: SearchQuery, db_path: str) -> list[COTSItem]:
    """
    Search for COTS parts in the database based on a query and constraints.
    """
    global _global_engine_cache
    if db_path not in _global_engine_cache:
        _global_engine_cache[db_path] = create_engine(f"sqlite:///{db_path}")

    engine = _global_engine_cache[db_path]
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

    # Fetch catalog metadata for reproducibility
    catalog_version = None
    bd_warehouse_commit = None
    generated_at = None

    with Session(engine) as session:
        meta_stmt = (
            session.query(CatalogMetadataORM)
            .order_by(CatalogMetadataORM.id.desc())
            .limit(1)
        )
        meta_result = meta_stmt.first()
        if meta_result:
            catalog_version = meta_result.catalog_version
            bd_warehouse_commit = meta_result.bd_warehouse_commit
            generated_at = (
                meta_result.generated_at.isoformat()
                if meta_result.generated_at
                else None
            )

    # Emit search event
    emit_event(
        COTSSearchEvent(
            query=query.query or str(query.constraints),
            results_count=len(results),
            catalog_version=catalog_version,
            bd_warehouse_commit=bd_warehouse_commit,
            generated_at=generated_at,
        )
    )

    return results
