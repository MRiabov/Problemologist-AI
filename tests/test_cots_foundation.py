import sqlite3

import pytest
from sqlalchemy import create_engine
from sqlalchemy.orm import Session

from shared.cots.database.init import init_db
from shared.cots.database.models import COTSItemORM
from shared.cots.models import COTSItem, SearchQuery
from shared.cots.runtime import search_parts
from shared.type_checking import type_check


@type_check
def test_cots_item_validation():
    data = {
        "part_id": "M3-HEX-10",
        "name": "M3 Hex Bolt 10mm",
        "category": "fastener",
        "unit_cost": 0.15,
        "weight_g": 1.2,
        "import_recipe": "Box(3, 3, 10)",
        "metadata": {"material": "steel", "finish": "zinc"},
    }
    item = COTSItem(**data)
    assert item.part_id == "M3-HEX-10"
    assert item.category == "fastener"


@type_check
def test_cots_item_invalid_category():
    data = {
        "part_id": "M3-HEX-10",
        "name": "M3 Hex Bolt 10mm",
        "category": "invalid",
        "unit_cost": 0.15,
        "weight_g": 1.2,
        "import_recipe": "Box(3, 3, 10)",
        "metadata": {},
    }
    with pytest.raises(ValueError):
        COTSItem(**data)


@type_check
def test_init_db(tmp_path):
    db_file = tmp_path / "test_parts.db"
    init_db(str(db_file))

    assert db_file.exists()

    # Verify table exists using sqlite3
    conn = sqlite3.connect(db_file)
    cursor = conn.cursor()
    cursor.execute(
        "SELECT name FROM sqlite_master WHERE type='table' AND name='parts';"
    )
    table = cursor.fetchone()
    assert table is not None
    assert table[0] == "parts"

    # Verify columns
    cursor.execute("PRAGMA table_info(parts);")
    columns = {col[1]: col[2] for col in cursor.fetchall()}
    assert "part_id" in columns
    assert "name" in columns
    assert "category" in columns
    assert "unit_cost" in columns
    assert "weight_g" in columns
    assert "import_recipe" in columns
    assert "metadata" in columns

    conn.close()


@type_check
def test_cots_min_size_filtering(tmp_path):
    db_file = tmp_path / "test_parts_search.db"
    init_db(str(db_file))
    engine = create_engine(f"sqlite:///{db_file}")

    with Session(engine) as session:
        small_part = COTSItemORM(
            part_id="small_part",
            name="Small Part",
            category="fastener",
            unit_cost=0.1,
            weight_g=1.0,
            import_recipe="part = Box(5, 5, 5)",
            metadata_dict={"bbox": {"min": [0, 0, 0], "max": [5, 5, 5]}},
        )
        large_part = COTSItemORM(
            part_id="large_part",
            name="Large Part",
            category="fastener",
            unit_cost=0.5,
            weight_g=10.0,
            import_recipe="part = Box(20, 20, 20)",
            metadata_dict={"bbox": {"min": [0, 0, 0], "max": [20, 20, 20]}},
        )
        session.add_all([small_part, large_part])
        session.commit()

    query = SearchQuery(query="", constraints={"min_size": 10}, limit=10)
    results = search_parts(query, str(db_file))

    ids = [r.part_id for r in results]
    assert "small_part" not in ids
    assert "large_part" in ids
