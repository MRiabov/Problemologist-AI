from pathlib import Path

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


def search(query: str, directory: str = "docs") -> str:
    """
    Simple substring search MVP. Searches docs/ and .agent/skills/.
    Returns top matches formatted as a string.

    TODO: Replace with proper embedding-based RAG for semantic search.
    Current implementation uses naive substring matching which fails for
    complex queries, synonyms, and semantic similarity. Consider integrating
    a vector store (e.g., ChromaDB, FAISS) with sentence embeddings.
    """
    query = query.lower()

    # Load from default docs
    docs = load_docs(directory)

    # Also load from skills
    skills_docs = load_docs(".agent/skills")

    all_docs = docs + skills_docs + STUBS

    matches = []
    for doc in all_docs:
        # Check title and content
        title_hits = doc["title"].lower().count(query)
        content_hits = doc["content"].lower().count(query)

        if title_hits > 0 or content_hits > 0:
            # Simple scoring
            score = title_hits * 10 + content_hits
            matches.append((score, doc))

    # Sort by score descending
    matches.sort(key=lambda x: x[0], reverse=True)

    # Get top 5
    top_matches = [m[1] for m in matches[:5]]

    if not top_matches:
        return f"No relevant documentation found for: '{query}' in {directory}/ or .agent/skills/"

    result = []
    for doc in top_matches:
        # Try to find the match context
        content = doc["content"]
        idx = content.lower().find(query)
        start = max(0, idx - 200)
        end = min(len(content), idx + 800)

        snippet = content[start:end]
        if start > 0:
            snippet = "..." + snippet
        if end < len(content):
            snippet = snippet + "..."

        result.append(f"=== {doc['title']} ({doc['path']}) ===\n{snippet}")

    return "\n\n".join(result)
