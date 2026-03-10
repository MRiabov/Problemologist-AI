## 1. Learning Objective

Test lateral transfer over 1 meter distance. The benchmark should move a 40 mm steel sphere from the left platform to the right goal zone without leaving simulation bounds.

## 2. Geometry

- Ground plane at Z=0 made from HDPE.
- Left and right support platforms centered near X=-0.5 m and X=+0.5 m.
- Two guide rails along the X axis around Y=+/-0.06 m to contain the sphere.
- Build zone limited to X=[-0.58, 0.58], Y=[-0.1, 0.1], Z=[0, 0.18].

## 3. Objectives

- Goal zone AABB near X=+0.5 m, Y=0, Z=0.08 m.
- Failure if the sphere exits simulation bounds.
- Gravity-only transfer; no active drive mechanism.

## 4. Randomization

- Static variation on platform height between 0.07 m and 0.09 m.
- Static variation on rail Y offset between -0.01 m and +0.01 m.
- Runtime jitter on the sphere start position.

## 5. Implementation Notes

- Use Build123d primitives.
- Ensure every part has `PartMetadata` or `CompoundMetadata`.
- The final script must contain an explicit `submit_for_review` call.
