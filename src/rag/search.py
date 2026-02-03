from pathlib import Path
from typing import List, Dict, Any, Tuple
import logging
import pickle
import hashlib
import os

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

try:
    from langchain_openai import OpenAIEmbeddings
    from src.agent.utils.config import Config
    HAS_LANGCHAIN = True
except ImportError:
    HAS_LANGCHAIN = False

logger = logging.getLogger(__name__)

# Hardcoded stubs for build123d when real docs are missing
STUBS = [
    {
        "title": "Box",
        "content": "Box(length, width, height) creates a rectangular prism. Example: Box(10, 20, 30)",
        "path": "docs/stubs.md",
    },
    {
        "title": "Cylinder",
        "content": "Cylinder(radius, height) creates a cylinder. Example: Cylinder(5, 20)",
        "path": "docs/stubs.md",
    },
    {
        "title": "Sphere",
        "content": "Sphere(radius) creates a sphere. Example: Sphere(10)",
        "path": "docs/stubs.md",
    },
    {
        "title": "Fillet",
        "content": "fillet(edges, radius) rounds selected edges. Example: fillet(my_obj.edges(), radius=1)",
        "path": "docs/stubs.md",
    },
    {
        "title": "Extrude",
        "content": "extrude(face, amount) creates a solid from a face. Example: extrude(my_face, amount=10)",
        "path": "docs/stubs.md",
    },
]

CACHE_DIR = Path(".cache")
CACHE_FILE = CACHE_DIR / "semantic_search.pkl"

def load_docs(directory: str) -> list[dict[str, str]]:
    """
    Simple doc loader that reads .md and .py files.
    """
    docs = []
    dir_path = Path(directory)
    if not dir_path.exists():
        return docs

    for ext in ["*.md", "*.py"]:
        for filename in dir_path.rglob(ext):
            try:
                docs.append(
                    {
                        "title": filename.name,
                        "content": filename.read_text(encoding="utf-8"),
                        "path": str(filename),
                    }
                )
            except Exception as e:
                print(f"Error loading {filename}: {e}")
    return docs


class SemanticSearch:
    def __init__(self, directory: str = "docs"):
        self.directory = directory
        self.chunks: List[Dict[str, Any]] = []
        self.embeddings: List[List[float]] = []
        self.matrix: Any = None
        self._initialized = False
        self._current_content_hash = ""

        if HAS_LANGCHAIN and Config.OPENAI_API_KEY:
            self.encoder = OpenAIEmbeddings(
                openai_api_key=Config.OPENAI_API_KEY,
                openai_api_base=Config.OPENAI_API_BASE
            )
        else:
            self.encoder = None
            if not HAS_LANGCHAIN:
                logger.warning("Langchain missing. Falling back to keyword search.")
            elif not Config.OPENAI_API_KEY:
                 logger.warning("OpenAI credentials not found. Falling back to keyword search.")

    def _chunk_docs(self, docs: List[Dict[str, str]]):
        """Splits documents into smaller chunks for better retrieval."""
        chunked_data = []
        for doc in docs:
            text = doc["content"]
            # Simple paragraph splitting
            paragraphs = text.split("\n\n")
            current_chunk = []
            current_length = 0

            for p in paragraphs:
                if current_length + len(p) > 1000:
                    # Emit chunk
                    chunk_text = "\n\n".join(current_chunk)
                    if chunk_text.strip():
                        chunked_data.append({
                            "title": doc["title"],
                            "path": doc["path"],
                            "content": chunk_text
                        })
                    current_chunk = [p]
                    current_length = len(p)
                else:
                    current_chunk.append(p)
                    current_length += len(p)

            # Leftover
            if current_chunk:
                chunk_text = "\n\n".join(current_chunk)
                if chunk_text.strip():
                    chunked_data.append({
                        "title": doc["title"],
                        "path": doc["path"],
                        "content": chunk_text
                    })
        return chunked_data

    def _get_content_hash(self) -> str:
        """Calculates a hash of the content state (mtimes of files)."""
        hash_md5 = hashlib.md5()
        # Directories to check
        dirs_to_check = [Path(self.directory), Path(".agent/skills")]

        for dir_path in dirs_to_check:
            if not dir_path.exists():
                continue
            # We use sorted walk to be deterministic
            for root, _, files in os.walk(dir_path):
                for name in sorted(files):
                     if not (name.endswith(".md") or name.endswith(".py")):
                         continue
                     filepath = Path(root) / name
                     try:
                         stat = filepath.stat()
                         # Update hash with path, modification time and size
                         hash_md5.update(str(filepath).encode('utf-8'))
                         hash_md5.update(str(stat.st_mtime).encode('utf-8'))
                         hash_md5.update(str(stat.st_size).encode('utf-8'))
                     except OSError:
                         pass
        return hash_md5.hexdigest()

    def _load_cache(self, expected_hash: str) -> bool:
        if not HAS_NUMPY:
            return False

        if CACHE_FILE.exists():
            try:
                with open(CACHE_FILE, 'rb') as f:
                    data = pickle.load(f)

                    cached_hash = data.get('hash', '')
                    if cached_hash != expected_hash:
                         logger.debug("Cache expired.")
                         return False

                    self.chunks = data.get('chunks', [])
                    self.embeddings = data.get('embeddings', [])

                    if self.chunks and self.embeddings and len(self.chunks) == len(self.embeddings):
                        self.matrix = np.array(self.embeddings)
                        # Pre-normalize
                        norm = np.linalg.norm(self.matrix, axis=1, keepdims=True)
                        norm[norm == 0] = 1e-10
                        self.matrix = self.matrix / norm
                        return True
            except Exception as e:
                logger.warning(f"Failed to load cache: {e}")
        return False

    def _save_cache(self, content_hash: str):
        if not CACHE_DIR.exists():
            try:
                CACHE_DIR.mkdir(parents=True, exist_ok=True)
            except Exception:
                return

        try:
            with open(CACHE_FILE, 'wb') as f:
                pickle.dump({
                    'hash': content_hash,
                    'chunks': self.chunks,
                    'embeddings': self.embeddings
                }, f)
        except Exception as e:
            logger.warning(f"Failed to save cache: {e}")

    def initialize(self, force_refresh=False):
        current_hash = self._get_content_hash()

        if self._initialized and not force_refresh:
            if current_hash == self._current_content_hash:
                return

        self._current_content_hash = current_hash

        # Try load from cache first
        if not force_refresh and self._load_cache(current_hash):
             self._initialized = True
             return

        # Load from default docs
        docs = load_docs(self.directory)
        # Also load from skills
        skills_docs = load_docs(".agent/skills")

        all_docs = docs + skills_docs + STUBS

        self.chunks = self._chunk_docs(all_docs)

        if not self.chunks:
            return

        if self.encoder and HAS_NUMPY:
            texts = [c["content"] for c in self.chunks]
            try:
                self.embeddings = self.encoder.embed_documents(texts)
                self.matrix = np.array(self.embeddings)

                # Normalize matrix for cosine similarity
                norm = np.linalg.norm(self.matrix, axis=1, keepdims=True)
                # Avoid division by zero
                norm[norm == 0] = 1e-10
                self.matrix = self.matrix / norm

                # Save cache
                self._save_cache(current_hash)

            except Exception as e:
                logger.error(f"Failed to generate embeddings: {e}")
                self.encoder = None # Fallback

        self._initialized = True

    def search(self, query: str, top_k: int = 5) -> str:
        self.initialize()

        if not self.chunks:
             return f"No documentation found in {self.directory}/ or .agent/skills/"

        # Fallback to keyword search if no encoder or numpy
        if not self.encoder or not HAS_NUMPY or self.matrix is None:
            return self._keyword_search(query, top_k)

        try:
            query_vec = self.encoder.embed_query(query)
            query_vec = np.array(query_vec)
            # Normalize query
            query_norm = np.linalg.norm(query_vec)
            if query_norm > 0:
                query_vec = query_vec / query_norm

            # Compute cosine similarity: dot product of normalized vectors
            scores = np.dot(self.matrix, query_vec)

            # Get top k indices
            top_indices = np.argsort(scores)[::-1][:top_k]

            results = []
            seen_keys = set()

            for idx in top_indices:
                score = scores[idx]
                chunk = self.chunks[idx]
                if score < 0.3: # Threshold for relevance
                    continue

                # Deduplicate
                key = (chunk["title"], chunk["path"])
                if key in seen_keys:
                    continue
                seen_keys.add(key)

                results.append(f"=== {chunk['title']} ({chunk['path']}) [Score: {score:.2f}] ===\n{chunk['content']}")
                if len(results) >= top_k:
                    break

            if not results:
                # Try keyword fallback if semantic search yields nothing useful (low scores)
                return self._keyword_search(query, top_k)

            return "\n\n".join(results)

        except Exception as e:
            logger.error(f"Semantic search failed: {e}. Falling back to keyword.")
            return self._keyword_search(query, top_k)

    def _keyword_search(self, query: str, top_k: int = 5) -> str:
        query = query.lower()
        matches = []

        # We search in chunks to be consistent
        for chunk in self.chunks:
            content = chunk["content"]
            title = chunk["title"]

            title_hits = title.lower().count(query)
            content_hits = content.lower().count(query)

            if title_hits > 0 or content_hits > 0:
                score = title_hits * 10 + content_hits
                matches.append((score, chunk))

        matches.sort(key=lambda x: x[0], reverse=True)

        # Deduplicate by (title, path) to ensure variety in results
        seen_keys = set()
        top_matches = []
        for score, chunk in matches:
            key = (chunk["title"], chunk["path"])
            if key not in seen_keys:
                top_matches.append(chunk)
                seen_keys.add(key)
            if len(top_matches) >= top_k:
                break

        if not top_matches:
            return f"No relevant documentation found for: '{query}'"

        result = []
        for chunk in top_matches:
             result.append(f"=== {chunk['title']} ({chunk['path']}) ===\n{chunk['content']}")

        return "\n\n".join(result)


# Singleton instance
_SEARCH_ENGINE = None

def search(query: str, directory: str = "docs") -> str:
    """
    Searches documentation using semantic search (if available) or keyword search.

    Args:
        query: The search query string.
        directory: The directory to search for documentation (default: "docs").

    Returns:
        A formatted string containing top matching documentation snippets.
    """
    global _SEARCH_ENGINE
    if _SEARCH_ENGINE is None:
        _SEARCH_ENGINE = SemanticSearch(directory)

    if _SEARCH_ENGINE.directory != directory:
         _SEARCH_ENGINE = SemanticSearch(directory)

    return _SEARCH_ENGINE.search(query)
