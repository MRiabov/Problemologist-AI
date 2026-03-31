import pytest

from worker_heavy.activities.heavy_tasks import _collect_submission_artifacts

pytestmark = pytest.mark.integration_p1


def test_submission_artifact_collection_preserves_bucketed_render_paths(tmp_path):
    root = tmp_path

    bucketed_render_paths = [
        root / "renders" / "benchmark_renders" / "render_a.png",
        root / "renders" / "engineer_renders" / "render_b_depth.png",
        root / "renders" / "final_preview_renders" / "render_c_segmentation.png",
    ]
    for path in bucketed_render_paths:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"not-a-real-image-but-good-enough-for-path-collection")

    render_manifest_path = root / "renders" / "render_manifest.json"
    render_manifest_path.parent.mkdir(parents=True, exist_ok=True)
    render_manifest_path.write_text("{}", encoding="utf-8")

    artifacts = _collect_submission_artifacts(root)

    expected_paths = {
        "renders/benchmark_renders/render_a.png",
        "renders/engineer_renders/render_b_depth.png",
        "renders/final_preview_renders/render_c_segmentation.png",
    }
    assert expected_paths.issubset(set(artifacts.render_paths)), artifacts.render_paths
    assert expected_paths.issubset(set(artifacts.render_blobs_base64)), (
        artifacts.render_blobs_base64
    )
    assert "renders/render_manifest.json" in artifacts.render_blobs_base64, (
        artifacts.render_blobs_base64
    )
