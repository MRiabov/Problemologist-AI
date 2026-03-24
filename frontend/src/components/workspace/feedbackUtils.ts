export interface ParsedFeedbackDraft {
  topic: string | null;
  comment: string;
}

const FEEDBACK_PREFIX_RE = /^\[(.+?)\]\s*([\s\S]*)$/;

export function splitFeedbackComment(comment: string | null | undefined): ParsedFeedbackDraft {
  const trimmed = comment?.trim() ?? "";
  if (!trimmed) {
    return { topic: null, comment: "" };
  }

  const match = trimmed.match(FEEDBACK_PREFIX_RE);
  if (!match) {
    return { topic: null, comment: trimmed };
  }

  return {
    topic: match[1]?.trim() || null,
    comment: match[2]?.trim() || "",
  };
}

export function combineFeedbackComment(topic: string | null, comment: string): string {
  const trimmedComment = comment.trim();
  if (!topic) {
    return trimmedComment;
  }
  if (!trimmedComment) {
    return `[${topic}]`;
  }
  return `[${topic}] ${trimmedComment}`;
}
