from shared.workers.filesystem.backend import LocalFilesystemBackend


def test_local_write_overwrite(tmp_path):
    session_dir = tmp_path / "sessions"
    backend = LocalFilesystemBackend.create("sess", base_dir=session_dir)

    # 1. Write first time
    res = backend.write("test.txt", "content1")
    assert not res.error
    assert (session_dir / "sess" / "test.txt").read_text() == "content1"

    # 2. Write second time without overwrite (should fail)
    res = backend.write("test.txt", "content2", overwrite=False)
    assert res.error
    assert "already exists" in res.error
    assert (session_dir / "sess" / "test.txt").read_text() == "content1"

    # 3. Write second time with overwrite (should succeed)
    res = backend.write("test.txt", "content2", overwrite=True)
    assert not res.error
    assert (session_dir / "sess" / "test.txt").read_text() == "content2"
