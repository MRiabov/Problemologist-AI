---
name: build123d_docs
description: Instructions for accessing and using the build123d condensed documentation.
---

# build123d Documentation Reader

Use this skill to quickly reference `build123d` syntax and best practices from the local documentation database.

## Instructions

1. **Locate Reference**: The primary condensed reference is located at `docs/llms.txt`.
2. **Search Documentation**: If you are unsure about a specific function or selector, read `docs/llms.txt` using the `view_file` tool.
3. **Verify Syntax**: Always check the reference before implementing complex boolean operations or selector chains to avoid common pitfalls like context shadowing.
4. **Condensed Knowledge**: This documentation is optimized for LLMs to reduce tokens while maintaining high accuracy of syntax.

## Key Reference Locations

- **Main Reference**: `docs/llms.txt`
- **Detailed Examples**: `docs/examples.md`
- **Future RAG Storage**: `docs/knowledge_base/` (planned)

## Usage Example

"I need to remember the arguments for `PolarLocations`. I will check `docs/llms.txt` via `view_file` to verify."
