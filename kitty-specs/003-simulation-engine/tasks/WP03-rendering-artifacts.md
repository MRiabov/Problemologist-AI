---
work_package_id: WP03
title: Rendering & Artifact Storage
lane: planned
dependencies: []
subtasks: [T013, T014, T015, T016, T017, T018]
---

# WP03: Rendering & Artifact Storage

## Objective

Enable the simulation engine to produce visual proof of the simulation (MP4 video) and persist data (MJCF, Video, Report) to cloud storage (S3/MinIO).

## Context

Visual feedback is critical for users to trust the agent's work. Since this runs in a headless worker, we must perform offscreen rendering and encode it efficiently.
Artifacts must be immediately uploaded to S3 to provide a permanent URL.

## Implementation Guide

### T013: Implement VideoRenderer

**File**: `src/worker/simulation/renderer.py`

1. Class `VideoRenderer`.
2. `__init__(self, model, data, width=640, height=480)`:
   - Initialize `mujoco.Renderer(model, height, width)`.
3. `render_frame(self) -> numpy.ndarray`:
   - Call `self.renderer.update_scene(data)`.
   - Return `self.renderer.render()`.
   - **Performance Note**: Rendering every physics step (500Hz) is wasteful. We should render at 30Hz or 60Hz.

### T014: Implement Frame Capture Logic

**File**: `src/worker/simulation/loop.py`

1. Update `SimulationLoop`.
2. Add `fps_render = 30`.
3. Calculate `render_interval = 1 / fps_render`.
4. Inside `step()` loop:
   - Check if `current_time - last_render_time >= render_interval`.
   - If yes, `frame = renderer.render_frame()`.
   - Store frame in a list `self.frames`.
   - **Caution**: Storing raw frames in memory (RAM) can result in OOM if simulation is long.
   - **Better**: Write frames immediately to a pipe (see T015).

### T015: Implement FFmpeg Encoding Pipeline

**File**: `src/worker/simulation/renderer.py`

1. Use `ffmpeg-python` or `subprocess`.
2. Method `initialize_encoder(self, output_path)`:
   - Open a subprocess to `ffmpeg -f rawvideo -pix_fmt rgb24 -s 640x480 -r 30 -i - ... output.mp4`.
3. Method `add_frame(self, frame)`:
   - Write bytes to `process.stdin.write(frame.tobytes())`.
4. Method `close(self)`:
   - Close stdin, wait for process to finish.

### T016: Implement S3 Upload

**File**: `src/worker/utils/storage.py` (or existing storage util)

1. Ensure we have a configured `minio.Minio` client.
2. Method `upload_file(file_path: Path, bucket: str, object_name: str) -> str`:
   - Perform upload.
   - Return pre-signed URL (valid for e.g. 7 days) OR public URL if bucket is public.
   - Use public URL preference for this project if configured.

### T017: Integrate Rendering into SimulationLoop

**File**: `src/worker/simulation/loop.py`

1. Pass `storage_client` to `SimulationLoop.run()`.
2. During run, write frames to FFmpeg.
3. On completion (Success/Fail):
   - Finalize video.
   - Upload `video.mp4` -> S3.
   - Upload `scene.xml` -> S3 (useful for replay).
   - Return `video_url` in the result object.

### T018: Add Tests

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
