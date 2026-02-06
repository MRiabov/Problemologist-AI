import os
import structlog
from datetime import datetime

logger = structlog.get_logger(__name__)

def submit_for_review(notes: str = "") -> bool:
    """
    Marks the current design as ready for human review.
    """
    logger.info("submit_for_review", notes_preview=notes[:50])
    
    try:
        review_file = "REVIEW_REQUEST.md"
        timestamp = datetime.now().isoformat()
        
        content = f"""# Review Request

**Submitted at**: {timestamp}
**Session ID**: {os.getenv('SESSION_ID', 'unknown')}

## Designer Notes

{notes if notes else 'No notes provided.'}
"""
        
        with open(review_file, "w") as f:
            f.write(content)
            
        logger.info("review_request_created", file=review_file)
        return True
    except Exception as e:
        logger.error("submission_failed", error=str(e))
        return False