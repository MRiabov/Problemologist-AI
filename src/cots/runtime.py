from sqlalchemy import create_engine, or_
from sqlalchemy.orm import Session

from .database.models import COTSItemORM
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
            if "max_weight" in query.constraints:
                stmt = stmt.filter(
                    COTSItemORM.weight_g <= float(query.constraints["max_weight"])
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

    return results
