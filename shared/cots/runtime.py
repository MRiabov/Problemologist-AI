from sqlalchemy import create_engine, or_
from sqlalchemy.orm import Session

from shared.observability.events import emit_event
from shared.observability.schemas import COTSSearchEvent

from .database.models import CatalogMetadataORM, COTSItemORM
from .models import COTSItem, SearchQuery


def search_parts(query: SearchQuery, db_path: str) -> list[COTSItem]:
    """
    Search for COTS parts in the database based on a query and constraints.
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
            if "max_weight_g" in query.constraints:
                stmt = stmt.filter(
                    COTSItemORM.weight_g <= float(query.constraints["max_weight_g"])
                )
            if "max_cost" in query.constraints:
                stmt = stmt.filter(
                    COTSItemORM.unit_cost <= float(query.constraints["max_cost"])
                )
            if "category" in query.constraints:
                stmt = stmt.filter(
                    COTSItemORM.category == query.constraints["category"]
                )
            if "min_size" in query.constraints:
                # Assuming metadata contains bbox or params.size
                # This is more complex to filter in SQL directly if it's in JSON
                # For MVP, we might skip complex JSON filtering or do it in-memory
                pass

        # Limit results
        stmt = stmt.limit(query.limit)

        orm_items = stmt.all()
        for item in orm_items:
            # In-memory filtering for min_size
            if query.constraints and "min_size" in query.constraints:
                min_size = float(query.constraints["min_size"])
                meta = item.metadata_dict
                if meta and "bbox" in meta:
                    bbox = meta["bbox"]
                    # Calculate dimensions from bbox
                    dims = [
                        bbox["max"][i] - bbox["min"][i] for i in range(3)
                    ]
                    max_dim = max(dims)
                    if max_dim < min_size:
                        continue

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
