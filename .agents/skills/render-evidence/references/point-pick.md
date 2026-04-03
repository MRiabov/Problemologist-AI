# Point-Pick Queries

`pick_preview_pixel(...)` resolves one screen-space pixel against one immutable bundle snapshot and returns a typed world-space hit record. The helper is not a generic image picker: it needs the bundle identity and the camera/view context that produced the render.

Import it from the preview namespace:

```python
from utils.preview import pick_preview_pixel, pick_preview_pixels
```

## Use When

- You need the world point under a pixel.
- You need to know what object a click lands on in a specific bundle.
- You need a bundle-local answer instead of guessing from image bytes.

## Contract

- Query the immutable bundle-local snapshot that produced the render.
- Keep `bundle_id`, `revision`, and `scene_hash` aligned with the artifact.
- Use one request object per pick, or pass the scalar fields directly for a single call.
- Do not infer world coordinates from raw pixels alone.

## Request Shape

Required fields:

- `bundle_path`
- `pixel_x`
- `pixel_y`
- `image_width`
- `image_height`
- `view_index`

Useful disambiguators:

- `orbit_pitch`
- `orbit_yaw`
- `bundle_id`
- `manifest_path`
- `rendering_type` when the runtime exposes it

Prefer the structured request object when you already have bundle metadata:

```python
pick_preview_pixel(
    RenderBundlePointPickRequest(
        bundle_path="renders/engineer_renders/example_bundle",
        pixel_x=312,
        pixel_y=184,
        image_width=1024,
        image_height=768,
        view_index=0,
        orbit_pitch=45.0,
        orbit_yaw=45.0,
        manifest_path="renders/engineer_renders/example_bundle/render_manifest.json",
    )
)
```

For a quick scalar call, pass the same inputs directly:

```python
pick_preview_pixel(
    bundle_path="renders/engineer_renders/example_bundle",
    pixel_x=312,
    pixel_y=184,
    image_width=1024,
    image_height=768,
    view_index=0,
    orbit_pitch=45.0,
    orbit_yaw=45.0,
    manifest_path="renders/engineer_renders/example_bundle/render_manifest.json",
)
```

## Expected Result

The helper should return, or make recoverable, fields like:

- bundle identity
- view identity or `view_index`
- pixel coordinates
- ray origin and ray direction
- world point or miss state
- object identity fields when available

If the call misses, still read the ray data and the bundle metadata. A miss is useful only if the snapshot and camera parameters are replayable.

## Fail Closed When

- The snapshot hash does not match the queried artifact.
- The bundle path is stale or missing.
- The query mixes a newer render with an older snapshot.
- The request does not identify the bundle or view clearly enough to be replayable.

## Relationship To Other Tools

- Use the runtime's general visual-inspection path for general visual review.
- Use depth or segmentation first when they answer the question more cheaply.
- Use point-pick queries only when pixel-to-world or identity-at-pixel is required.
- Use `pick_preview_pixels(...)` when you need more than one pixel resolved against the same snapshot.

## Example Shape

The exact runtime signature may differ, but the intent must stay the same: resolve a click against the bundle that produced the evidence with explicit bundle, pixel, and image-dimension inputs.

## Common Mistakes

- Passing a `(x, y)` tuple and omitting `image_width` / `image_height`.
- Using the wrong `view_index` for the rendered frame.
- Querying a later bundle revision than the one that produced the screenshot.
- Treating the helper as a coordinate guesser instead of a replayable ray-pick.
