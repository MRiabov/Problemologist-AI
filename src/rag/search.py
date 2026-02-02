import glob
import os

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
    if not os.path.exists(directory):
        return docs

    for ext in ["*.md", "*.py"]:
        for filename in glob.glob(os.path.join(directory, "**", ext), recursive=True):
            try:
                with open(filename, encoding="utf-8") as f:
                    docs.append(
                        {
                            "title": os.path.basename(filename),
                            "content": f.read(),
                            "path": filename,
                        }
                    )
            except Exception as e:
                print(f"Error loading {filename}: {e}")
    return docs


def search(query: str, directory: str = "docs") -> str:
    """
    Simple substring search MVP. Returns top 3 matches formatted as a string.
    """
    query = query.lower()
    docs = load_docs(directory)

    # Always include stubs for now to ensure results
    all_docs = docs + STUBS

    matches = []
    for doc in all_docs:
        # Check title and content
        if query in doc["title"].lower() or query in doc["content"].lower():
            # Simple scoring: count occurrences
            score = doc["title"].lower().count(query) * 2 + doc[
                "content"
            ].lower().count(query)
            matches.append((score, doc))

    # Sort by score descending
    matches.sort(key=lambda x: x[0], reverse=True)

    # Get top 3
    top_matches = [m[1] for m in matches[:3]]

    if not top_matches:
        return "No relevant documentation found for: " + query

    result = []
    for doc in top_matches:
        result.append(
            f"=== {doc['title']} ({doc['path']}) ===\n{doc['content'][:500]}..."
        )

    return "\n\n".join(result)
