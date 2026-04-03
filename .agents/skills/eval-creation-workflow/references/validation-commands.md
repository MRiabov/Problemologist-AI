# Validation Commands

Use these commands while creating or repairing eval seeds.

## Validate One Seed

```bash
uv run scripts/validate_eval_seed.py \
  --agent benchmark_coder \
  --task-id bc-001-example
```

## Run One Eval

```bash
uv run dataset/evals/run_evals.py \
  --agent benchmark_coder \
  --task-id bc-001-example \
  --limit 1 \
  --concurrency 1 \
  --verbose --log-level INFO
```

## Validate an Agent Slice

```bash
uv run scripts/validate_eval_seed.py \
  --agent benchmark_reviewer
```

## Check Seeded Entry in Logs

```bash
rg "eval_seed_workspace_applied|start_node=" logs/evals/run_evals.log logs/evals/controller.log
```

## Hash a Manifest Input

```bash
sha256sum dataset/data/seed/artifacts/benchmark_reviewer/br-001-example/script.py
```
