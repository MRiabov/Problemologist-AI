import pytest
from src.cots.core import PartIndex, PartSummary, PartPreview, Part
from src.cots.providers.base import PartProvider
from typing import List


class MockProvider(PartProvider):
    def __init__(self, name: str):
        self.name = name
        self.parts = {
            f"{self.name}:bolt1": Part(id=f"{self.name}:bolt1", factory=lambda: "bolt"),
            f"{self.name}:nut1": Part(id=f"{self.name}:nut1", factory=lambda: "nut"),
        }

    def search(self, query: str) -> List[PartSummary]:
        return [
            PartSummary(id=pid, name=pid.split(":")[1], provider=self.name)
            for pid in self.parts.keys()
            if query in pid
        ]

    def get_part(self, part_id: str) -> Part:
        return self.parts[part_id]

    def get_preview(self, part_id: str) -> PartPreview:
        return PartPreview(
            id=part_id, image_path=f"{part_id}.png", description=f"Mock {part_id}"
        )


def test_part_index_registration():
    index = PartIndex()
    provider = MockProvider("fasteners")
    index.register_provider("fasteners", provider)
    assert "fasteners" in index.providers


def test_part_index_search():
    index = PartIndex()
    index.register_provider("p1", MockProvider("p1"))
    index.register_provider("p2", MockProvider("p2"))

    results = index.search("bolt")
    assert len(results) == 2
    assert any(r.id == "p1:bolt1" for r in results)
    assert any(r.id == "p2:bolt1" for r in results)


def test_part_index_preview_routing():
    index = PartIndex()
    index.register_provider("p1", MockProvider("p1"))

    preview = index.preview("p1:bolt1")
    assert preview.id == "p1:bolt1"
    assert preview.image_path == "p1:bolt1.png"


def test_part_index_invalid_id():
    index = PartIndex()
    with pytest.raises(ValueError, match="Invalid part ID"):
        index.preview("invalid_id")


def test_part_index_unknown_provider():
    index = PartIndex()
    with pytest.raises(ValueError, match="No provider registered"):
        index.preview("unknown:id")
