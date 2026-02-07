---
work_package_id: WP03
title: Rendering & Artifact Storage
lane: "for_review"
dependencies: []
base_branch: main
base_commit: 064b45154507c1584b8b086c1c9d93b6dc52019f
created_at: '2026-02-07T06:32:01.549481+00:00'
subtasks: [T014, T015, T016, T017, T018, T019]
shell_pid: "150427"
agent: "Gemini-CLI"
---

## WP03: Rendering & Artifact Storage

## Objective

Enable the simulation engine to produce visual proof of the simulation (MP4 video) and persist data (MJCF, Video, Report) to cloud storage (S3/MinIO).

## Context

Visual feedback is critical for users to trust the agent's work. Since this runs in a headless worker, we must perform offscreen rendering and encode it efficiently.
Artifacts must be immediately uploaded to S3 to provide a permanent URL.

## Implementation Guide

### T014: Implement Renderer

**File**: `src/worker/simulation/renderer.py`

1. Class `Renderer`.
2. `__init__(self, model, data, width=640, height=480)`:
   - Initialize `mujoco.Renderer(model, height, width)`.
3. `render_frame(self, camera=None) -> numpy.ndarray`:
   - Call `self.renderer.update_scene(data, camera=camera)`.
   - Return `self.renderer.render()`.
4. `render_24_views(self) -> list[numpy.ndarray]`:
   - Implement logic to orbit the camera (8 horizontal angles x 3 vertical levels).
   - Return a list of 24 images.

### T015: Implement Conditional Frame Capture Logic

**File**: `src/worker/simulation/loop.py`

1. Update `SimulationLoop`.
2. Add `fps_render = 30`.
3. Calculate `render_interval = 1 / fps_render`.
4. Inside `step()` loop:
   - **Only if `render=True` in call**:
   - Check if `current_time - last_render_time >= render_interval`.
   - If yes, `frame = renderer.render_frame()`.
   - Store frame in a list `self.frames`.
   - **Caution**: Storing raw frames in memory (RAM) can result in OOM if simulation is long.
   - **Better**: Write frames immediately to a pipe (see T016).

### T016: Implement Artifact Bundling

**File**: `src/worker/simulation/renderer.py`

1. **Video**: Use `ffmpeg-python` to encode frames to `output.mp4`.
2. **24-View Bundle**: Save 24 images to a temporary folder, then create a zip or tarball `preview_bundle.zip`.
3. Method `save_artifacts(self, output_dir: Path)`:
   - Ensure video and preview bundle are written to `/renders/`.

### T017: Implement S3 Upload

**File**: `src/worker/utils/storage.py` (or existing storage util)

1. Ensure we have a configured `minio.Minio` client.
2. Method `upload_file(file_path: Path, bucket: str, object_name: str) -> str`:
   - Perform upload.
   - Return pre-signed URL (valid for e.g. 7 days) OR public URL if bucket is public.
   - Use public URL preference for this project if configured.

### T018: Integrate Rendering into SimulationLoop

**File**: `src/worker/simulation/loop.py`

1. Pass `storage_client` to `SimulationLoop.run()`.
2. During run, write frames to FFmpeg (only if `render=True`).
3. On completion (Success/Fail):
   - Finalize video (if rendering).
   - Upload `video.mp4` -> S3 (if rendering).
   - Upload `scene.xml` -> S3 (always useful for replay).
   - Return `video_url` (or None) in the result object.

### T019: Add Tests

1. **Mock S3**: Use `moto` or a simple mock class that prints "Uploaded X" and returns a dummy URL.
2. **Video Test**:
   - Run a short sim (0.5s).
   - Assert `output.mp4` exists.
   - Check file size > 1KB.
   - (Optional) Use `ffprobe` to verify duration matches simulation time.

## Validation

- [ ] Simulation produces a valid `.mp4` file.
- [ ] Mock upload returns a URL.
- [ ] Memory usage remains stable during rendering (no leaks).

## Risks

- **Headless GPU**: Rendering might fail on servers without a display or GPU.
  - *Mitigation*: MuJoCo `egl` backend or `osmesa` might be needed. Use `MUJOCO_GL=egl` env var. If no GPU, `osmesa` (software rendering) is slow but works.
  - **Action**: Add instructions to README/Dockerfile to install `libgl1-mesa-glx` `libosmesa6-dev`.

## Activity Log

- 2026-02-07T06:32:01Z – Gemini-CLI – shell_pid=150427 – lane=doing – Assigned agent via workflow command
- 2026-02-07T06:47:16Z – Gemini-CLI – shell_pid=150427 – lane=for_review – Ready for review: Implemented offscreen rendering with Renderer class, integrated frame capture into SimulationLoop, added S3 storage client, and verified with video generation tests.
