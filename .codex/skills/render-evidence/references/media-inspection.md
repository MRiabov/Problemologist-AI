# Media Inspection

## Use the Runtime's Media-Inspection Path When

- You need to inspect `.png`, `.jpg`, `.jpeg`, or `.mp4` evidence.
- You need actual pixel-level or frame-level inspection.
- A render already exists and you want to judge it instead of generating a new one.

## Choose the Right Modality

- RGB: general scene understanding, layout, and rough correctness.
- Depth: clearance, collision, and separation checks.
- Segmentation: object identity, overlap, repeated instances, and ownership checks.
- Video: motion direction, stability, timing, and capture behavior.

## Procedure

1. Open the actual artifact path.
2. Inspect one clear question at a time.
3. If the task is about motion, inspect the first useful frames before changing geometry.
4. If the task is about a rear view, confirm the camera orientation before assuming a mirror flip.
5. If the task requires review evidence, record what was inspected and why.

## What To Confirm

- The media belongs to the intended bundle and revision.
- The view matches the question being asked.
- Geometry, clearance, identity, or motion agree with the contract.
- The image or video actually answers the question better than a summary would.

## Common Mistakes

- Reading a directory listing instead of the pixels.
- Using filenames as proof.
- Mixing stale and current evidence.
- Treating `simulation_result.json` or a text summary as a substitute for media inspection.
