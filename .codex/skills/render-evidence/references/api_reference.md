# Render Query API Reference

Use these references for the worker-light render-query helpers:

- Preferred import surface:

```python
from utils.preview import (
    list_render_bundles,
    objectives_geometry,
    pick_preview_pixel,
    pick_preview_pixels,
    preview,
    preview_drawing,
    query_render_bundle,
)

from utils.visualize import preview as visualize_preview
```

- `pick_preview_pixel(...)` and `pick_preview_pixels(...)`: see [point-pick.md](point-pick.md) for the required inputs, replayable call shape, and returned hit fields.
- `list_render_bundles(...)` and `query_render_bundle(...)`: see [bundle-history.md](bundle-history.md) for bundle selection and metadata lookup.

All render queries must stay bundle-local and replayable. Do not infer pixel-to-world coordinates from filenames, raw image bytes, or an unnamed latest render.
