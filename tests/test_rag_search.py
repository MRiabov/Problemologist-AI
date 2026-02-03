import pytest
from unittest.mock import patch, MagicMock
from pathlib import Path
import src.rag.search as search_module
from src.rag.search import search
import logging

@pytest.fixture(autouse=True)
def reset_search_engine():
    search_module._SEARCH_ENGINE = None
    cache_file = search_module.CACHE_FILE
    if cache_file.exists():
        try:
            cache_file.unlink()
        except OSError:
            pass
    yield
    search_module._SEARCH_ENGINE = None
    if cache_file.exists():
        try:
            cache_file.unlink()
        except OSError:
            pass

@pytest.fixture
def mock_docs_dir(tmp_path):
    docs_dir = tmp_path / "docs"
    docs_dir.mkdir()
    (docs_dir / "test1.md").write_text(
        "# Test Document 1\nThis is a test document about Box components.",
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

@pytest.fixture
def mock_load_docs():
    with patch("src.rag.search.load_docs") as mock:
        def side_effect(directory):
            # Use 'skills' to catch relative or absolute paths to skills dir
            if "skills" in str(directory):
                return []

            docs = []
            dir_path = Path(directory)
            if not dir_path.exists():
                return docs

            for ext in ["*.md", "*.py"]:
                for filename in dir_path.rglob(ext):
                    docs.append({
                        "title": filename.name,
                        "content": filename.read_text(encoding="utf-8"),
                        "path": str(filename),
                    })
            return docs

        mock.side_effect = side_effect
        yield mock

def test_load_docs(mock_docs_dir):
    from src.rag.search import load_docs as real_load_docs
    docs = real_load_docs(mock_docs_dir)
    assert len(docs) == 3

def test_load_docs_missing_dir():
    from src.rag.search import load_docs as real_load_docs
    docs = real_load_docs("/non/existent/dir")
    assert docs == []

def test_search_hit_filesystem(mock_docs_dir, mock_load_docs):
    result = search("Box", directory=mock_docs_dir)
    assert "test1.md" in result
    assert "Box" in result
    assert "docs/stubs.md" in result

def test_search_hit_stubs(mock_docs_dir, mock_load_docs):
    result = search("Sphere", directory=mock_docs_dir)
    assert "test3.md" in result
    assert "Sphere(radius)" in result

def test_search_no_hits(mock_docs_dir, mock_load_docs):
    result = search("NonExistentComponent123", directory=mock_docs_dir)
    assert "No relevant documentation found" in result

def test_search_scoring(mock_docs_dir, mock_load_docs):
    result = search("Cylinder", directory=mock_docs_dir)
    assert result.startswith("=== Cylinder (docs/stubs.md) ===")

def test_search_empty_query(mock_docs_dir, mock_load_docs):
    result = search("", directory=mock_docs_dir)
    assert "===" in result
