import { memo } from "react";
import { Terminal, AlertCircle, ThumbsUp, ThumbsDown } from "lucide-react";
import { ThoughtBlock } from "./ThoughtBlock";
import { ActionCard } from "./ActionCard";
import { HighlightedContent } from "./HighlightedContent";
import type { TraceResponse } from "../../api/generated/models/TraceResponse";
import type { AssetResponse } from "../../api/generated/models/AssetResponse";
import type { ContextItem } from "../../context/EpisodeContext";
import { TraceType } from "../../api/generated/models/TraceType";

interface TraceListProps {
  traces: TraceResponse[] | undefined;
  assets: AssetResponse[] | undefined;
  theme: string;
  showReasoning: boolean;
  onAssetClick: (id: string | null) => void;
  addToContext: (item: ContextItem) => void;
  onShowFeedback: (traceId: number, score: number) => void;
}

function extractReasoningFromPhaseLog(content: string): string {
  const thoughtMatches = [...content.matchAll(/thought_\d+['"]?\s*:\s*['"]([^'"]+)['"]/g)]
    .map((m) => m[1]?.trim())
    .filter(Boolean) as string[];
  if (thoughtMatches.length > 0) {
    return thoughtMatches.join("\n");
  }

  const reasoningMatch = content.match(/reasoning['"]?\s*[:=]\s*['"]([^'"]+)['"]/i);
  if (reasoningMatch?.[1]) {
    return reasoningMatch[1].trim();
  }

  // Fallback: keep original text when no structured thought/reasoning is found.
  return content;
}

export const TraceList = memo(({
  traces,
  assets,
  theme,
  showReasoning,
  onAssetClick,
  addToContext,
  onShowFeedback
}: TraceListProps) => {

  if (!traces || traces.length === 0) {
    return (
      <div className="flex flex-col items-center justify-center py-20 gap-2 opacity-10 h-full">
         <Terminal className="h-8 w-8" />
         <span className="text-[10px] uppercase font-bold tracking-widest">Awaiting interaction...</span>
      </div>
    );
  }

  return (
    <>
      {traces.map((trace, index) => {
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
              const nextTool = traces
                .slice(index + 1)
                .find((t) => t.trace_type === TraceType.TOOL_START);
              const nodeName = trace.name || "step";
              title = nextTool?.name
                ? `Reasoning after ${nodeName} before ${nextTool.name}`
                : `Reasoning after ${nodeName}`;
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
              // Render node transition logs as reasoning when explicitly enabled.
              const isPhaseLog =
                !!trace.name &&
                (trace.content.includes("Starting task phase:")
                  || trace.content.includes("Completed task phase:"));
              if (showReasoning && isPhaseLog) {
                const stableDuration = (String(trace.id).split('').reduce((acc: number, char: string) => acc + char.charCodeAt(0), 0) % 5) + 1;
                const title = `Reasoning around ${trace.name}`;
                const reasoningContent = extractReasoningFromPhaseLog(trace.content);
                return (
                  <ThoughtBlock
                    key={trace.id}
                    duration={stableDuration}
                    content={reasoningContent}
                    title={title}
                    testId="reasoning-span"
                  />
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
                    <div className="flex items-center gap-2 mt-1 opacity-0 group-hover/msg:opacity-100 transition-opacity">
                        {isLastLlmEnd ? (
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
                            </div>
                        ) : (
                            <button
                                onClick={() => onShowFeedback(trace.id, 1)}
                                className="text-[10px] text-muted-foreground hover:text-foreground flex items-center gap-1"
                            >
                                <AlertCircle className="h-3 w-3" />
                                Feedback
                            </button>
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
