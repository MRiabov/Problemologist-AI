Integration mock scenarios live here as one file per `INT-###` case.

- Name each scenario file `INT-###.yaml`.
- Keep large `write_file` payloads in adjacent fixture files and reference them with `tool_args.content_file`.
- `content_file` paths resolve relative to this directory.
- The loader fails closed on invalid IDs, duplicate IDs, missing fixture files, or invalid transcript schema.
