import { useRef, useEffect } from "react";
import { ScrollArea } from "../../components/ui/scroll-area";
import type { TraceResponse } from "../../api/generated/models/TraceResponse";
import { Terminal } from "lucide-react";

interface ReasoningTracesProps {
  traces?: TraceResponse[];
  isRunning?: boolean;
}

export default function ReasoningTraces({
  traces,
  isRunning
}: ReasoningTracesProps) {
  const scrollRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new traces arrive
  useEffect(() => {
    if (traces && traces.length > 0) {
      const timeout = setTimeout(() => {
        const scrollContainer = scrollRef.current?.querySelector('[data-radix-scroll-area-viewport]');
        if (scrollContainer) {
          scrollContainer.scrollTop = scrollContainer.scrollHeight;
        }
      }, 100);
      return () => clearTimeout(timeout);
    }
  }, [traces]);

  return (
    <div className="flex flex-col h-full bg-card/30 border-r border-border">
      {/* Content Area */}
      <div className="flex-1 overflow-hidden flex flex-col relative">
        <ScrollArea className="flex-1 bg-black/20" ref={scrollRef}>
            <div className="p-4 font-mono text-[11px] leading-relaxed">
                {traces && traces.length > 0 ? (
                    traces.map(trace => (
                        <div key={trace.id} className="space-y-1 mb-3 border-b border-white/5 pb-2 last:border-0 hover:bg-white/5 p-1 rounded transition-colors group">
                            <div className="flex justify-between text-[9px] text-muted-foreground">
                                <span className="text-primary/70 font-bold">[{new Date(trace.created_at).toLocaleTimeString()}]</span>
                                <span className="opacity-50 uppercase group-hover:opacity-100 transition-opacity">Trace: {trace.id}</span>
                            </div>
                            <div className="text-muted-foreground break-all opacity-90 whitespace-pre-wrap">
                                {typeof trace.raw_trace === 'string' 
                                    ? trace.raw_trace 
                                    : JSON.stringify(trace.raw_trace, null, 2)}
                            </div>
                        </div>
                    ))
                ) : (
                    <div className="flex flex-col items-center justify-center h-full text-muted-foreground/30 py-20 gap-2">
                       <Terminal className="h-8 w-8 opacity-20" />
                       <span className="text-[10px] uppercase font-bold tracking-widest">No traces available</span>
                    </div>
                )}
                
                {/* Visual indicator for active streaming at the bottom */}
                {isRunning && (
                    <div className="mt-2 text-primary animate-pulse pl-1 opacity-50 text-[10px]">_</div>
                )}
            </div>
        </ScrollArea>
      </div>
    </div>
  );
}
