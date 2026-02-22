import { memo } from "react";
import { Terminal, AlertCircle } from "lucide-react";
import { ThoughtBlock } from "./ThoughtBlock";
import { ActionCard } from "./ActionCard";
import { HighlightedContent } from "./HighlightedContent";
import type { TraceResponse } from "../../api/generated/models/TraceResponse";
import type { AssetResponse } from "../../api/generated/models/AssetResponse";
import type { ContextItem } from "../../context/EpisodeContext";

interface TraceListProps {
  traces: TraceResponse[] | undefined;
  assets: AssetResponse[] | undefined;
  theme: string;
  onAssetClick: (id: string | null) => void;
  addToContext: (item: ContextItem) => void;
  onShowFeedback: () => void;
}

export const TraceList = memo(({
  traces,
  assets,
  theme,
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
      {traces.map(trace => {
          const type = trace.trace_type as string;
          if (type === 'llm_thought' || type === 'thought') {
            const stableDuration = (trace.id.split('').reduce((acc, char) => acc + char.charCodeAt(0), 0) % 5) + 1;
            return <ThoughtBlock key={trace.id} duration={stableDuration} content={trace.content || ""} />;
          }
          if (type === 'tool_start') {
              return (
                <ActionCard
                  key={trace.id}
                  trace={trace}
                  assets={assets}
                  setActiveArtifactId={onAssetClick}
                />
              );
          }
          if (type === 'llm_end' && trace.content) {
            return (
                <div key={trace.id} className="relative group/msg">
                    <HighlightedContent
                      content={trace.content}
                      language="markdown"
                      theme={theme}
                      assets={assets}
                      addToContext={addToContext}
                      setActiveArtifactId={onAssetClick}
                    />
                    <div className="flex items-center gap-2 mt-1 opacity-0 group-hover/msg:opacity-100 transition-opacity">
                        <button
                            onClick={onShowFeedback}
                            className="text-[10px] text-muted-foreground hover:text-foreground flex items-center gap-1"
                        >
                            <AlertCircle className="h-3 w-3" />
                            Feedback
                        </button>
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
