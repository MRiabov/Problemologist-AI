import pytest
from unittest.mock import MagicMock, patch
import numpy as np
from pathlib import Path
from src.rag.search import search, SemanticSearch, CACHE_FILE

@pytest.fixture
def mock_docs_dir(tmp_path):
    docs_dir = tmp_path / "docs"
    docs_dir.mkdir()
    (docs_dir / "doc1.md").write_text("This is document one.", encoding="utf-8")
    (docs_dir / "doc2.md").write_text("This is document two.", encoding="utf-8")
    return str(docs_dir)

@patch("src.rag.search.Config")
@patch("src.rag.search.OpenAIEmbeddings")
def test_semantic_search_ranking(mock_embeddings_cls, mock_config, mock_docs_dir):
    mock_config.OPENAI_API_KEY = "fake-key"
    mock_config.OPENAI_API_BASE = "fake-base"
    mock_embeddings = MagicMock()
    mock_embeddings_cls.return_value = mock_embeddings
    mock_embeddings.embed_query.return_value = [1.0, 0.0]

    def side_effect_embed_documents(texts):
        vectors = []
        for text in texts:
            if "document one" in text:
                vectors.append([0.9, 0.1])
            elif "document two" in text:
                vectors.append([0.1, 0.9])
            else:
                vectors.append([0.0, 1.0])
        return vectors

    mock_embeddings.embed_documents.side_effect = side_effect_embed_documents

    searcher = SemanticSearch(directory=mock_docs_dir)
    results = searcher.search("query")

    assert "doc1.md" in results
    lines = results.split('\n')
    assert "doc1.md" in lines[0]
    assert "Score: 0.99" in lines[0]

@patch("src.rag.search.Config")
@patch("src.rag.search.OpenAIEmbeddings")
def test_search_fallback_integration(mock_embeddings_cls, mock_config, mock_docs_dir):
    mock_config.OPENAI_API_KEY = "fake-key"
    mock_embeddings = MagicMock()
    mock_embeddings_cls.return_value = mock_embeddings
    mock_embeddings.embed_query.return_value = [1, 0]
    mock_embeddings.embed_documents.return_value = [[1, 0]] * 7

    result = search("something", directory=mock_docs_dir)
    assert isinstance(result, str)

@patch("src.rag.search.Config")
@patch("src.rag.search.OpenAIEmbeddings")
def test_cache_invalidation(mock_embeddings_cls, mock_config, mock_docs_dir):
    mock_config.OPENAI_API_KEY = "fake-key"
    mock_embeddings = MagicMock()
    mock_embeddings_cls.return_value = mock_embeddings
    mock_embeddings.embed_query.return_value = [1.0, 0.0]
    mock_embeddings.embed_documents.return_value = [[0.9, 0.1]] * 10 # return something

    searcher = SemanticSearch(directory=mock_docs_dir)

    # First search - should generate cache
    searcher.search("query")
    first_hash = searcher._current_content_hash
    assert first_hash
    assert mock_embeddings.embed_documents.call_count == 1

    # Modify file
    # Ensure mtime changes (sometimes fast tests run in same second)
    import time
    time.sleep(0.01) # Small sleep, or force mtime update

    p = Path(mock_docs_dir) / "doc1.md"
    p.write_text("Modified content for cache invalidation test", encoding="utf-8")

    # Search again
    searcher.search("query")
    second_hash = searcher._current_content_hash

    assert second_hash != first_hash
    assert mock_embeddings.embed_documents.call_count == 2 # Called again!
