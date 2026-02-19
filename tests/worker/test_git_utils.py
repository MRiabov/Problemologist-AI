from worker_light.utils.git import (
    abort_merge,
    commit_all,
    complete_merge,
    get_repo_status,
    has_merge_conflicts,
    init_workspace_repo,
    resolve_conflict_ours,
)


def test_git_conflict_resolution(tmp_path):
    # 1. Setup repo
    repo = init_workspace_repo(tmp_path)
    default_branch = repo.active_branch.name
    file_path = tmp_path / "conflict.txt"
    file_path.write_text("Base content")
    commit_all(tmp_path, "Initial commit")

    # 2. Create a branch 'feature'
    repo.git.checkout("-b", "feature")
    file_path.write_text("Feature content")
    commit_all(tmp_path, "Feature commit")

    # 3. Go back to main and change file
    repo.git.checkout(default_branch)
    file_path.write_text("Main content")
    commit_all(tmp_path, "Main commit")

    # 4. Merge feature into main -> Conflict
    try:
        repo.git.merge("feature")
    except Exception:
        pass  # Conflict expected

    status = get_repo_status(tmp_path)
    assert status["is_merging"] is True
    assert "conflict.txt" in status["conflicts"]

    # 5. Resolve conflict (Ours)
    resolved = resolve_conflict_ours(tmp_path, "conflict.txt")
    assert resolved is True

    status = get_repo_status(tmp_path)
    assert "conflict.txt" not in status["conflicts"]

    # 6. Complete merge
    commit = complete_merge(tmp_path, "Merge resolved")
    assert commit is not None

    status = get_repo_status(tmp_path)
    assert status["is_merging"] is False
    assert file_path.read_text() == "Main content"


def test_git_abort_merge(tmp_path):
    # 1. Setup repo
    repo = init_workspace_repo(tmp_path)
    default_branch = repo.active_branch.name
    file_path = tmp_path / "conflict.txt"
    file_path.write_text("Base content")
    commit_all(tmp_path, "Initial commit")

    repo.git.checkout("-b", "feature")
    file_path.write_text("Feature content")
    commit_all(tmp_path, "Feature commit")

    repo.git.checkout(default_branch)
    file_path.write_text("Main content")
    commit_all(tmp_path, "Main commit")

    try:
        repo.git.merge("feature")
    except Exception:
        pass

    assert has_merge_conflicts(tmp_path) is True

    # Abort
    aborted = abort_merge(tmp_path)
    assert aborted is True
    assert has_merge_conflicts(tmp_path) is False
    assert (tmp_path / ".git" / "MERGE_HEAD").exists() is False
    assert file_path.read_text() == "Main content"
