# Point-Pick Queries

## Use When

- You need the world point under a pixel.
- You need to know what object a click lands on in a specific bundle.
- You need a bundle-local answer instead of guessing from image bytes.

## Contract

- Query the immutable bundle-local snapshot that produced the render.
- Keep `bundle_id`, `revision`, and `scene_hash` aligned with the artifact.
- Use one request object per pick.
- Do not infer world coordinates from raw pixels alone.

## Expected Result

The helper should return, or make recoverable, fields like:

- bundle identity
- view identity or `view_index`
- pixel coordinates
- ray origin and ray direction
- world point or miss state
- object identity fields when available

## Fail Closed When

- The snapshot hash does not match the queried artifact.
- The bundle path is stale or missing.
- The query mixes a newer render with an older snapshot.
- The request does not identify the bundle or view clearly enough to be replayable.

## Relationship To Other Tools

- Use the runtime's general visual-inspection path for general visual review.
- Use depth or segmentation first when they answer the question more cheaply.
- Use point-pick queries only when pixel-to-world or identity-at-pixel is required.

## Example Shape

```text
pick_preview_pixel(bundle_id=..., view_index=0, pixel=(312, 184))
```

The exact runtime signature may differ, but the intent must stay the same: resolve a click against the bundle that produced the evidence.
