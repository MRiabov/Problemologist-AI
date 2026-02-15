---
name: spec-kitty-management
description: Guidance on how to correctly run spec-kitty commands using the project's virtual environment.
---

# Spec-Kitty Management

You'll use spec-kitty very often in my prompts. You need to use it with a `venv`, so something like:

`source .venv/bin/activate && spec-kitty [...]`

## Feature management

You'll definitely need to use a `--feature` parameter in almost any call. If the user says "WP1 in spec 009", here "spec" is equal to "feature", and you need to look up the full feature name, e.g. `009-fluids-deformable-materials`, or else --feature will fail with:

```txt
Error: Invalid feature slug format: 009
Expected format: ###-feature-name (e.g., 020-my-feature)
```

You easily do this with doing `ls kitty-specs`.
