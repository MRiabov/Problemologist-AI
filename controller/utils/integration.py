import re

from shared.models.schemas import EpisodeMetadata

_INT_ID_PATTERN = re.compile(r"\bINT-\d+\b", re.IGNORECASE)


def infer_integration_test_id(*texts: str | None) -> str | None:
    """Extract canonical INT-xxx identifier from any candidate text."""
    for text in texts:
        if not text:
            continue
        match = _INT_ID_PATTERN.search(text)
        if match:
            return match.group(0).upper()
    return None


def apply_integration_test_metadata(
    metadata: EpisodeMetadata,
    *,
    is_integration_test: bool,
    task: str | None = None,
    session_id: str | None = None,
) -> EpisodeMetadata:
    """Mark episode metadata as integration-generated and derive INT id when possible."""
    if not is_integration_test:
        return metadata

    metadata.is_integration_test = True
    metadata.integration_test_id = metadata.integration_test_id or infer_integration_test_id(
        task, session_id, metadata.prompt
    )
    return metadata
