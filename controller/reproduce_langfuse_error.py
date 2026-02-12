import os
import uuid
import logging
from langfuse import Langfuse

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Ensure we use the internal docker host
os.environ["LANGFUSE_HOST"] = "http://langfuse-server:3000"


def test_span_with_valid_tags():
    logger.info("TEST: Span with valid tags=['test']")
    try:
        langfuse = Langfuse()
        span = langfuse.start_span(name="span-valid-tags")
        logger.info(f"Created span trace_id: {span.trace_id}")

        span.update_trace(tags=["test"])

        span.end()
        langfuse.flush()
        logger.info("Span with valid tags flushed.")
    except Exception as e:
        logger.error(f"Span with valid tags failed: {e}")


if __name__ == "__main__":
    test_span_with_valid_tags()
