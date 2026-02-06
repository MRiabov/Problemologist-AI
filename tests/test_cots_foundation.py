import os
import sqlite3
import pytest
from src.cots.models import COTSItem
from src.cots.database.init import init_db


def test_cots_item_validation():
    data = {
        "part_id": "M3-HEX-10",
        "name": "M3 Hex Bolt 10mm",
        "category": "fastener",
        "unit_cost": 0.15,
        "weight_g": 1.2,
        "import_recipe": "Box(3, 3, 10)",
        "metadata": {"material": "steel", "finish": "zinc"}
    }
    item = COTSItem(**data)
    assert item.part_id == "M3-HEX-10"
    assert item.category == "fastener"


def test_cots_item_invalid_category():
    data = {
        "part_id": "M3-HEX-10",
        "name": "M3 Hex Bolt 10mm",
        "category": "invalid",
        "unit_cost": 0.15,
        "weight_g": 1.2,
        "import_recipe": "Box(3, 3, 10)",
        "metadata": {}
    }
    with pytest.raises(ValueError):
        COTSItem(**data)


def test_init_db(tmp_path):
    db_file = tmp_path / "test_parts.db"
    init_db(str(db_file))
    
    assert db_file.exists()
    
    # Verify table exists using sqlite3
    conn = sqlite3.connect(db_file)
    cursor = conn.cursor()
    cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='parts';")
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
