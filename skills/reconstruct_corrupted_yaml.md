# Reconstruct Corrupted YAML Files

## Problem

YAML configuration files (objectives.yaml, assembly_definition.yaml) become corrupted with:

- Interleaved content from multiple files
- Parsing errors from malformed YAML structure
- Diff markers (@@) and partial content mixed together

## Solution Pattern

When standard read operations fail or return corrupted content:

1. **Use execute_command to inspect raw content**:

   ```bash
   cat /objectives.yaml
   head -30 /objectives.yaml
   ```

2. **Extract key information from corrupted content**:

   - Search for recognizable patterns (goal_zone, build_zone, etc.)
   - Identify the original structure despite corruption

3. **Reconstruct using write_file**:

   ```python
   write_file(path="/objectives.yaml", content=reconstructed_yaml, overwrite=True)
   ```

## When to Use

- YAML parsing errors appear
- Content appears interleaved with other file data
- Diff markers (@@) visible in file content
- Standard read_file returns malformed content

## Key Observation

The corrupted files can still contain extractable information. The goal is to understand the original structure and reconstruct a valid YAML file from scratch rather than trying to patch the corrupted version.

## Example from Session

- File: objectives.yaml was corrupted with assembly_definition content
- Fix: Used execute_command to see raw content, then write_file to reconstruct proper objectives.yaml with all original constraints (goal_zone, build_zone, forbid_zones, physics settings)
