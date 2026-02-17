from sqlalchemy import create_engine
from sqlalchemy.orm import Session

from shared.cots.database.models import COTSItemORM
from shared.cots.indexer import Indexer


def test_indexer_basic(tmp_path):
    db_path = tmp_path / "test_parts.db"
    indexer = Indexer(str(db_path))

    # Index only 1 item per class to be fast
    indexer.index_all(limit_per_class=1)

    assert db_path.exists()

    engine = create_engine(f"sqlite:///{db_path}")
    with Session(engine) as session:
        items = session.query(COTSItemORM).all()
        # We expect 9 classes * 1 item = 9 items
        assert len(items) == 9

        # Check one item
        nut = session.query(COTSItemORM).filter_by(category="fastener").first()
        assert nut is not None
        assert "HexNut" in nut.part_id
        assert nut.unit_cost == 0.05
        assert nut.weight_g > 0
        assert "import HexNut" in nut.import_recipe
        assert nut.metadata_dict.get("volume", 0) > 0


def test_metadata_extraction():
    indexer = Indexer(":memory:")
    from bd_warehouse.fastener import HexNut

    meta = indexer.extract_metadata(HexNut, "M6-1")
    assert meta.part_id == "HexNut_M6-1"
    assert meta.category == "fastener"
    assert meta.unit_cost == 0.05
    assert meta.bbox is not None


def test_recipe_generation():
    indexer = Indexer(":memory:")
    from bd_warehouse.fastener import HexNut

    recipe = indexer.generate_recipe(HexNut, {"size": "M6-1"})
    assert "from bd_warehouse.fastener import HexNut" in recipe
    assert "HexNut(size='M6-1')" in recipe
