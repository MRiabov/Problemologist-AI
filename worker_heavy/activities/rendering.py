import os
import tempfile
import uuid
import asyncio
from pathlib import Path

import numpy as np
import structlog
import vtk
from build123d import export_stl
from temporalio import activity

from shared.observability.storage import S3Client, S3Config
from shared.workers.loader import load_component_from_script as _load_component
from shared.workers.schema import HeavyRenderSnapshotParams, HeavyRenderSnapshotResponse

logger = structlog.get_logger(__name__)


def _extract_bundle(bundle_bytes: bytes, target_dir: Path):
    """Extract gzipped tarball to target directory using system tar."""
    import subprocess

    with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tf:
        tf.write(bundle_bytes)
        tf_path = tf.name
    try:
        subprocess.run(
            ["tar", "-zxf", tf_path, "-C", str(target_dir), "--no-same-owner"],
            check=True,
            capture_output=True,
            text=True,
        )
    except subprocess.CalledProcessError as spe:
        logger.error("tar_subprocess_failed", stderr=spe.stderr)
        raise RuntimeError(f"tar extraction failed: {spe.stderr}")
    finally:
        if Path(tf_path).exists():
            Path(tf_path).unlink()


@activity.defn(name="worker_render_snapshot")
async def render_snapshot_activity(params: HeavyRenderSnapshotParams) -> HeavyRenderSnapshotResponse:
    """Render a selection snapshot from a session bundle."""
    bundle_bytes = params.bundle_bytes
    script_path = params.script_path
    target_ids = params.target_ids
    view_matrix = params.view_matrix

    with tempfile.TemporaryDirectory() as tmpdir:
        root = Path(tmpdir)
        _extract_bundle(bundle_bytes, root)

        try:
            # Run the rendering logic in a thread to avoid blocking the loop
            image_key = await asyncio.to_thread(
                render_selection_snapshot,
                ids=target_ids,
                view_matrix=view_matrix,
                script_path=root / script_path,
            )
            return HeavyRenderSnapshotResponse(success=True, image_key=image_key)
        except Exception as e:
            logger.error("render_snapshot_activity_failed", error=str(e))
            return HeavyRenderSnapshotResponse(success=False, error=str(e))


def render_selection_snapshot(
    ids: list[str],
    view_matrix: list[list[float]],
    script_path: str | Path = "script.py",
) -> str:
    """
    Renders a snapshot of the component with highlighted features using VTK.
    Uploads the result to S3 and returns the object key.

    Args:
        ids: List of target IDs to highlight (e.g. ['face_0', 'part_1'])
        view_matrix: 4x4 camera view matrix (world-to-camera)
        script_path: Path to the script that builds the component

    Returns:
        S3 object key of the rendered PNG.
    """
    logger.info("rendering_selection_snapshot", ids=ids)
    script_p = Path(script_path)

    component = _load_component(script_p)

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp_path = Path(tmpdir)

        renderer = vtk.vtkRenderer()
        render_window = vtk.vtkRenderWindow()
        render_window.SetOffScreenRendering(1)
        render_window.AddRenderer(renderer)

        # 1. Add base geometry
        base_stl = tmp_path / "base.stl"
        export_stl(component, base_stl)

        reader = vtk.vtkSTLReader()
        reader.SetFileName(str(base_stl))

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(reader.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.7, 0.7, 0.7)  # Grey base
        actor.GetProperty().SetOpacity(
            0.6
        )  # Slight transparency to emphasize highlights
        renderer.AddActor(actor)

        # 2. Add Highlights
        all_faces = component.faces()
        # solids = component.solids()

        for target_id in ids:
            try:
                if target_id.startswith("face_"):
                    idx = int(target_id.split("_")[1])
                    if idx < len(all_faces):
                        face = all_faces[idx]
                        face_stl = tmp_path / f"face_{idx}.stl"
                        export_stl(face, face_stl)

                        f_reader = vtk.vtkSTLReader()
                        f_reader.SetFileName(str(face_stl))
                        f_mapper = vtk.vtkPolyDataMapper()
                        f_mapper.SetInputConnection(f_reader.GetOutputPort())

                        f_actor = vtk.vtkActor()
                        f_actor.SetMapper(f_mapper)
                        f_actor.GetProperty().SetColor(
                            1.0, 1.0, 0.0
                        )  # Yellow highlight
                        f_actor.GetProperty().SetLineWidth(2.0)
                        renderer.AddActor(f_actor)
                # Part highlighting could be added here similarly using component.solids()
            except Exception as e:
                logger.warning(f"failed_to_highlight_{target_id}", error=str(e))

        # 3. Configure Camera
        camera = renderer.GetActiveCamera()

        # VTK expects world-to-camera matrix for its internal state if set directly,
        # but setting position/focal/up is more conventional.

        # Convert view_matrix (world-to-camera) to camera pose (camera-to-world)
        m = np.array(view_matrix)
        inv_m = np.linalg.inv(m)

        eye = inv_m[:3, 3]
        # Look direction is negative Z in camera space
        forward = -inv_m[:3, 2]
        # Focal point is some distance along forward vector
        focal_point = eye + forward
        up = inv_m[:3, 1]

        camera.SetPosition(eye[0], eye[1], eye[2])
        camera.SetFocalPoint(focal_point[0], focal_point[1], focal_point[2])
        camera.SetViewUp(up[0], up[1], up[2])

        renderer.ResetCameraClippingRange()
        renderer.SetBackground(1.0, 1.0, 1.0)  # White background for reports

        # 4. Render
        render_window.SetSize(1024, 768)
        render_window.Render()

        w2i = vtk.vtkWindowToImageFilter()
        w2i.SetInput(render_window)
        w2i.Update()

        png_path = tmp_path / "snapshot.png"
        writer = vtk.vtkPNGWriter()
        writer.SetFileName(str(png_path))
        writer.SetInputConnection(w2i.GetOutputPort())
        writer.Write()

        # 5. Upload to S3
        s3_config = S3Config(
            endpoint_url=os.getenv("S3_ENDPOINT"),
            access_key_id=os.getenv("S3_ACCESS_KEY", "minioadmin"),
            secret_access_key=os.getenv("S3_SECRET_KEY", "minioadmin"),
            bucket_name=os.getenv("ASSET_S3_BUCKET", "problemologist"),
            region_name=os.getenv("AWS_REGION", "us-east-1"),
        )
        s3_client = S3Client(s3_config)

        key = f"snapshots/{uuid.uuid4()}.png"
        s3_client.upload_file(str(png_path), key)

        logger.info("snapshot_uploaded", key=key)
        return key
