from controller.utils.integration import (
    apply_integration_test_metadata,
    infer_integration_test_id,
)
from shared.models.schemas import EpisodeMetadata


def test_infer_integration_test_id_from_task():
    assert infer_integration_test_id("Run scenario INT-173 with mock output") == "INT-173"


def test_apply_integration_test_metadata_sets_fields():
    meta = EpisodeMetadata(prompt="INT-045 test prompt")
    updated = apply_integration_test_metadata(
        meta,
        is_integration_test=True,
        task="Execute INT-045 flow",
        session_id="INT-045-abc123",
    )

    assert updated.is_integration_test is True
    assert updated.integration_test_id == "INT-045"


def test_apply_integration_test_metadata_noop_when_disabled():
    meta = EpisodeMetadata(prompt="INT-999 prompt")
    updated = apply_integration_test_metadata(
        meta,
        is_integration_test=False,
        task="INT-999",
        session_id="INT-999-x",
    )

    assert updated.is_integration_test is None
    assert updated.integration_test_id is None
