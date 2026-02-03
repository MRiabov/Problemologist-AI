import pytest

from src.rag.search import load_docs, search


@pytest.fixture
def mock_docs_dir(tmp_path):
    docs_dir = tmp_path / "docs"
    docs_dir.mkdir()

    # Create some dummy files
    (docs_dir / "test1.md").write_text(
        "# Test Document 1\nThis is a test document about UniqueBoxComponents.",
        encoding="utf-8",
    )
    (docs_dir / "test2.py").write_text(
        "def test():\n    return 'This is a python file with Cylinder reference.'",
        encoding="utf-8",
    )

    sub_dir = docs_dir / "subdir"
    sub_dir.mkdir()
    (sub_dir / "test3.md").write_text("Nested document about Sphere.", encoding="utf-8")

    return str(docs_dir)


def test_load_docs(mock_docs_dir):
    docs = load_docs(mock_docs_dir)
    assert len(docs) == 3
    titles = [d["title"] for d in docs]
    assert "test1.md" in titles
    assert "test2.py" in titles
    assert "test3.md" in titles

    # Check content of one
    test1 = next(d for d in docs if d["title"] == "test1.md")
    assert "Test Document 1" in test1["content"]


def test_load_docs_missing_dir():
    docs = load_docs("/non/existent/dir")
    assert docs == []


def test_search_hit_filesystem(mock_docs_dir):
    result = search("UniqueBoxComponents", directory=mock_docs_dir)
    assert "test1.md" in result
    assert "UniqueBoxComponents" in result


def test_search_hit_stubs(mock_docs_dir):
    # Sphere is in test3.md AND stubs
    result = search("Sphere", directory=mock_docs_dir)
    assert "test3.md" in result
    assert "Sphere(radius)" in result


def test_search_no_hits(mock_docs_dir):
    result = search("NonExistentComponent123", directory=mock_docs_dir)
    assert "No relevant documentation found" in result


def test_search_scoring(mock_docs_dir):
    # Cylinder is in test2.py (once) and stubs (once in title, once in content)
    # Stubs: Cylinder(radius, height) creates a cylinder. Example: Cylinder(5, 20)
    # Total score for stubs should be higher due to title match (2x) + content match
    result = search("Cylinder", directory=mock_docs_dir)
    # The first result (top match) should be from stubs because "Cylinder"
    # is in the title
    assert result.startswith("=== Cylinder (docs/stubs.md) ===")


def test_search_empty_query(mock_docs_dir):
    # Should probably return something or not crash
    result = search("", directory=mock_docs_dir)
    assert "===" in result  # Returns top results
