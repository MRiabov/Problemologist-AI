import pytest
from sqlalchemy import create_engine
from sqlalchemy.orm import Session

from shared.cots.database.models import Base, COTSItemORM
from shared.cots.models import SearchQuery
from shared.cots.runtime import search_parts


@pytest.fixture
def test_db(tmp_path):
    db_file = tmp_path / "test_parts.db"
    db_path = str(db_file)

    engine = create_engine(f"sqlite:///{db_path}")
    Base.metadata.create_all(engine)

    with Session(engine) as session:
        # Add some dummy parts
        p1 = COTSItemORM(
            part_id="test_bolt_m6",
            name="M6 Hex Bolt",
            category="fastener",
            unit_cost=0.5,
            weight_g=15.0,
            import_recipe="from bd_warehouse.fastener import HexBolt\nbolt = HexBolt(size='M6')",
            metadata_dict={"size": "M6"},
        )
        p2 = COTSItemORM(
            part_id="test_motor_nema17",
            name="NEMA 17 Stepper",
            category="motor",
            unit_cost=15.0,
            weight_g=350.0,
            import_recipe="from bd_warehouse.motor import Nema17\nmotor = Nema17()",
            metadata_dict={"type": "stepper"},
        )
        session.add_all([p1, p2])
        session.commit()

    return db_path


def test_search_by_name(test_db):
    query = SearchQuery(query="bolt")
    results = search_parts(query, test_db)
    assert len(results) == 1
    assert results[0].part_id == "test_bolt_m6"


def test_search_by_category(test_db):
    query = SearchQuery(query="motor")
    results = search_parts(query, test_db)
    assert len(results) == 1
    assert results[0].category == "motor"


def test_search_with_constraints(test_db):
    # Weight constraint
    query = SearchQuery(query="", constraints={"max_weight_g": 100})
    results = search_parts(query, test_db)
    assert len(results) == 1
    assert results[0].part_id == "test_bolt_m6"

    # Cost constraint
    query = SearchQuery(query="", constraints={"max_cost": 20})
    results = search_parts(query, test_db)
    assert len(results) == 2

    query = SearchQuery(query="", constraints={"max_cost": 1})
    results = search_parts(query, test_db)
    assert len(results) == 1
    assert results[0].part_id == "test_bolt_m6"


def test_search_limit(test_db):
    query = SearchQuery(query="", limit=1)
    results = search_parts(query, test_db)
    assert len(results) == 1
