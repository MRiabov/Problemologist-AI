import glob
import os
import inspect
from pathlib import Path
from typing import Callable, Type, Union


def get_docs_for(entity: Union[Type, Callable, str]) -> str:
    """
    Search for documentation and usage examples for a given entity (Type, Function, or name string).
    Searches in the `skills` directory (if available) for relevant markdown files.
    """
    query_name = ""
    if isinstance(entity, str):
        query_name = entity
    elif hasattr(entity, "__name__"):
        query_name = entity.__name__
    else:
        query_name = str(entity)

    # Try to find skills directory
    # Assuming standard structure where this file is in worker/utils/docs.py
    # and skills are in worker/skills or a mounted volume at /skills

    potential_skill_paths = [
        Path("/skills"),  # Mounted volume in container
        Path(__file__).parent.parent / "skills",  # Local dev
        Path("skills"),  # Current working dir
        Path(".agent/skills"),  # Repo root skills
    ]

    found_docs = []

    # 1. Check Python Docstring first
    if not isinstance(entity, str):
        doc = inspect.getdoc(entity)
        if doc:
            found_docs.append(f"### Docstring for `{query_name}`:\n{doc}\n")

    # 2. Search in skills files
    for skill_path in potential_skill_paths:
        if skill_path.exists():
            # Search for md files recursively
            for md_file in skill_path.rglob("*.md"):
                try:
                    content = md_file.read_text()
                    if query_name.lower() in content.lower():
                        # Extract relevant snippet (paragraph context)
                        paragraphs = content.split("\n\n")
                        relevant_paragraphs = []
                        for p in paragraphs:
                            if query_name.lower() in p.lower():
                                relevant_paragraphs.append(p.strip())
                                if len(relevant_paragraphs) >= 2:
                                    break

                        snippet = "\n\n".join(relevant_paragraphs)
                        if len(snippet) > 1000:
                            snippet = snippet[:1000] + "..."

                        found_docs.append(
                            f"### Found mention in `{md_file.name}`:\n(Path: {md_file})\n\n{snippet}\n"
                        )
                except Exception:
                    continue

    if not found_docs:
        return f"No documentation found for '{query_name}'."

    return "\n".join(found_docs)
