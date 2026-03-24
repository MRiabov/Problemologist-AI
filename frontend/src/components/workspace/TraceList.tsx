import { memo } from "react";
import { Terminal, AlertCircle, ThumbsUp, ThumbsDown } from "lucide-react";
import { ThoughtBlock } from "./ThoughtBlock";
import { ActionCard } from "./ActionCard";
import { HighlightedContent } from "./HighlightedContent";
import type { TraceResponse } from "../../api/generated/models/TraceResponse";
import type { AssetResponse } from "../../api/generated/models/AssetResponse";
import type { ContextItem } from "../../context/EpisodeContext";
import { TraceType } from "../../api/generated/models/TraceType";
import { EpisodeStatus } from "../../api/generated/models/EpisodeStatus";
import { cn } from "../../lib/utils";
import { splitFeedbackComment } from "./feedbackUtils";

interface TraceListProps {
  traces: TraceResponse[] | undefined;
  assets: AssetResponse[] | undefined;
  theme: string;
  showReasoning: boolean;
  reasoningRequired?: boolean;
  episodeStatus?: EpisodeStatus | null;
  onAssetClick: (id: string | null) => void;
  addToContext: (item: ContextItem) => void;
  onShowFeedback: (traceId: number, score: number) => void;
}

export const TraceList = memo(({
  traces,
  assets,
  theme,
  showReasoning,
  reasoningRequired = false,
  episodeStatus,
  onAssetClick,
  addToContext,
  onShowFeedback
}: TraceListProps) => {

  const traceList = traces ?? [];
  const hasReasoningTraces = traceList.some(
    (t) => t.trace_type === TraceType.LLM_END && !!t.name
  );
  const shouldShowTelemetryWarning =
    reasoningRequired &&
    !hasReasoningTraces &&
    episodeStatus !== undefined &&
    episodeStatus !== null &&
    [
      EpisodeStatus.RUNNING,
      EpisodeStatus.COMPLETED,
      EpisodeStatus.FAILED,
    ].includes(episodeStatus);
  const showReasoningTelemetryWarning =
    shouldShowTelemetryWarning;

  if (traceList.length === 0 && !showReasoningTelemetryWarning) {
    return (
      <div className="flex flex-col items-center justify-center py-20 gap-2 opacity-10 h-full">
         <Terminal className="h-8 w-8" />
         <span className="text-[10px] uppercase font-bold tracking-widest">Awaiting interaction...</span>
      </div>
    );
  }

  return (
    <>
      {showReasoningTelemetryWarning && (
        <div
          data-testid="reasoning-telemetry-warning"
          className="rounded-md border border-amber-400/30 bg-amber-500/10 px-3 py-2 text-[11px] text-amber-700 mb-3"
        >
          Reasoning telemetry is required for this run, but no reasoning traces have been persisted yet.
        </div>
      )}
      {traceList.map((trace, index) => {
          const type = trace.trace_type as string;
          const isLegacyThought = type === "llm_thought" || type === "thought";
          const isReasoningSpan =
            trace.trace_type === TraceType.LLM_END && !!trace.name;
          if (isLegacyThought || isReasoningSpan) {
            if (!showReasoning || !trace.content) {
              return null;
            }
            const stableDuration = (String(trace.id).split('').reduce((acc: number, char: string) => acc + char.charCodeAt(0), 0) % 5) + 1;
            let title = `Thought for ${stableDuration}s`;
            if (isReasoningSpan) {
              const nextTool = traceList
                .slice(index + 1)
                .find((t) => t.trace_type === TraceType.TOOL_START);
              const nodeName = trace.name || "step";
              const meta = (trace.metadata_vars || {}) as any;
              const reasoningStepIndex = meta.reasoning_step_index as number | undefined;
              const stepTitle = Number.isInteger(reasoningStepIndex)
                ? ` · step ${reasoningStepIndex}`
                : "";
              title = nextTool?.name
                ? `Reasoning after ${nodeName}${stepTitle} before ${nextTool.name}`
                : `Reasoning after ${nodeName}${stepTitle}`;
            }
            return (
              <ThoughtBlock
                key={trace.id}
                duration={stableDuration}
                content={trace.content}
                title={title}
                testId="reasoning-span"
              />
            );
          }
          if (trace.trace_type === TraceType.TOOL_START) {
              return (
                <ActionCard
                  key={trace.id}
                  trace={trace}
                  assets={assets}
                  setActiveArtifactId={onAssetClick}
                />
              );
          }
          if (trace.trace_type === TraceType.ERROR) {
              return (
                <div key={trace.id} className="flex items-start gap-2 p-3 bg-red-500/10 rounded-lg border border-red-500/20 my-2">
                    <AlertCircle className="h-4 w-4 text-red-500 shrink-0 mt-0.5" />
                    <div className="text-[11px] text-red-400 font-mono whitespace-pre-wrap">
                        {trace.content || "An error occurred."}
                    </div>
                </div>
              );
          }
          if (trace.trace_type === TraceType.EVENT && trace.name === "conversation_length_exceeded") {
              const meta = (trace.metadata_vars || {}) as any;
              const previousLength = Number(meta.previous_length);
              const compactedLength = Number(meta.compacted_length);
              const threshold = Number(meta.threshold);
              const hasCompactedLength = Number.isFinite(compactedLength) && compactedLength > 0;
              return (
                <div
                  key={trace.id}
                  data-testid="conversation-length-exceeded"
                  className="flex items-start gap-2 p-3 bg-amber-500/10 rounded-lg border border-amber-500/30 my-2"
                >
                  <AlertCircle className="h-4 w-4 text-amber-500 shrink-0 mt-0.5" />
                  <div className="text-[11px] text-amber-700 whitespace-pre-wrap">
                    Conversation length limit exceeded{Number.isFinite(threshold) ? ` (${threshold})` : ""}; compacted context
                    {Number.isFinite(previousLength) && hasCompactedLength
                      ? ` from ${previousLength} to ${compactedLength} characters.`
                      : "."}
                  </div>
                </div>
              );
          }
          if (trace.trace_type === TraceType.LOG && trace.content) {
              const meta = (trace.metadata_vars || {}) as any;
              if (meta.role === 'user' || trace.content.startsWith('User message:')) {
                  const displayContent = meta.message || trace.content.replace('User message: ', '');
                  return (
                    <div key={trace.id} data-testid="chat-message" className="flex justify-end mb-4">
                        <div className="max-w-[85%] bg-primary/10 rounded-2xl p-3 shadow-sm border border-primary/20">
                            <div className="text-[13px] leading-relaxed text-foreground whitespace-pre-wrap">
                                {displayContent}
                            </div>
                        </div>
                    </div>
                  );
              }
              // Other logs are intentionally not shown in the chat timeline.
              return null;
          }
          if (trace.trace_type === TraceType.LLM_END && trace.content) {
            if (trace.name) {
              return null;
            }
            const isLastLlmEnd = traces && traces.filter(t => t.trace_type === TraceType.LLM_END).pop()?.id === trace.id;
            const hasSavedFeedback =
              (trace.feedback_score !== null && trace.feedback_score !== undefined) ||
              !!trace.feedback_comment;
            const feedbackDraft = splitFeedbackComment(trace.feedback_comment);

            return (
                <div key={trace.id} data-testid="chat-message" className="relative group/msg">
                    <HighlightedContent
                      content={trace.content}
                      language="markdown"
                      theme={theme}
                      assets={assets}
                      addToContext={addToContext}
                      setActiveArtifactId={onAssetClick}
                    />
                    {hasSavedFeedback && (
                      <div
                        data-testid={`trace-feedback-summary-${trace.id}`}
                        className="mt-2 rounded-xl border border-border/60 bg-muted/20 px-3 py-2 space-y-1"
                      >
                        <div className="flex items-center gap-2 text-[10px] font-black uppercase tracking-widest text-muted-foreground">
                          <span
                            className={cn(
                              "rounded-full px-2 py-0.5 border",
                              trace.feedback_score === 1
                                ? "border-emerald-500/30 bg-emerald-500/10 text-emerald-600"
                                : trace.feedback_score === 0
                                  ? "border-red-500/30 bg-red-500/10 text-red-500"
                                  : "border-border/60 bg-background text-muted-foreground",
                            )}
                          >
                            {trace.feedback_score === 1 ? "Thumbs Up" : trace.feedback_score === 0 ? "Thumbs Down" : "Feedback"}
                          </span>
                          {feedbackDraft.topic && (
                            <span className="rounded-full border border-border/60 bg-background px-2 py-0.5 text-foreground">
                              {feedbackDraft.topic}
                            </span>
                          )}
                        </div>
                        {feedbackDraft.comment && (
                          <p className="text-[11px] leading-relaxed text-foreground whitespace-pre-wrap">
                            {feedbackDraft.comment}
                          </p>
                        )}
                      </div>
                    )}
                    <div className="flex items-center gap-2 mt-1 opacity-0 group-hover/msg:opacity-100 transition-opacity">
                        {isLastLlmEnd && (
                            <div className="flex items-center gap-1">
                                <button
                                    data-testid="chat-thumbs-up"
                                    onClick={() => onShowFeedback(trace.id, 1)}
                                    className="p-1 hover:bg-muted rounded-md text-muted-foreground hover:text-green-500 transition-colors"
                                    title="Thumbs Up"
                                >
                                    <ThumbsUp className="h-3.5 w-3.5" />
                                </button>
                                <button
                                    data-testid="chat-thumbs-down"
                                    onClick={() => onShowFeedback(trace.id, 0)}
                                    className="p-1 hover:bg-muted rounded-md text-muted-foreground hover:text-red-500 transition-colors"
                                    title="Thumbs Down"
                                >
                                    <ThumbsDown className="h-3.5 w-3.5" />
                                </button>
                                {hasSavedFeedback && (
                                  <span className="text-[10px] text-muted-foreground font-semibold pl-1">
                                    Edit saved feedback
                                  </span>
                                )}
                            </div>
                        )}
                    </div>
                </div>
            );
          }
          return null;
      })}
    </>
  );
});

TraceList.displayName = 'TraceList';
