import tempfile
from pathlib import Path

from build123d import Part

from shared.workers.filesystem.backend import LocalFilesystemBackend
from shared.workers.filesystem.router import (
    AccessMode,
    FilesystemRouter,
    MountPoint,
)
from worker_heavy.workbenches.base import (
    ManufacturingConfig,
    WorkbenchAnalyzer,
    WorkbenchResult,
)


def test_router_download_files():
    # Setup temporary directories
    with tempfile.TemporaryDirectory() as tmp_dir:
        tmp_path = Path(tmp_dir)

        # Local backend root
        local_root = tmp_path / "workspace"
        local_root.mkdir()

        # Mounted directory
        mount_root = tmp_path / "mount"
        mount_root.mkdir()

        # Create files
        (local_root / "local.txt").write_text("local content")
        (mount_root / "mounted.txt").write_text("mounted content")

        # Setup backend and router
        backend = LocalFilesystemBackend(local_root, "sess")

        mount = MountPoint(
            virtual_prefix="/mnt",
            local_path=mount_root,
            access_mode=AccessMode.READ_ONLY,
        )

        router = FilesystemRouter(local_backend=backend, mount_points=[mount])

        # Test download_files with mixed paths
        paths = ["local.txt", "/mnt/mounted.txt", "nonexistent.txt"]
        results = router.download_files(paths)

        assert len(results) == 3

        # Check local file
        res_local = next(r for r in results if r.path == "local.txt")
        assert res_local.content == b"local content"
        assert res_local.error is None

        # Check mounted file
        res_mounted = next(r for r in results if r.path == "/mnt/mounted.txt")
        assert res_mounted.content == b"mounted content"
        assert res_mounted.error is None

        # Check nonexistent file
        res_missing = next(r for r in results if r.path == "nonexistent.txt")
        assert res_missing.content is None
        assert res_missing.error is not None  # "file_not_found" or similar


def test_workbench_analyzer_protocol():
    # This test verifies that a function with quantity parameter satisfies
    # the protocol usage, checking runtime compatibility if we were to enforce it,
    # but mainly ensuring we updated the protocol definition in the codebase.

    def my_analyze(
        part: Part, config: ManufacturingConfig, quantity: int = 1
    ) -> WorkbenchResult:
        _ = (part, config, quantity)
        return WorkbenchResult(is_manufacturable=True, unit_cost=10.0, violations=[])

    # Check that WorkbenchAnalyzer has quantity in __call__
    import inspect

    sig = inspect.signature(WorkbenchAnalyzer.__call__)
    assert "quantity" in sig.parameters
