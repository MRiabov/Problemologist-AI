import { useRef, useEffect, useState } from "react";
import { ScrollArea } from "../../components/ui/scroll-area";
import type { TraceResponse } from "../../api/generated/models/TraceResponse";
import { Terminal, Send, Square, ThumbsUp, ThumbsDown, Check } from "lucide-react";
import { submitTraceFeedback } from "../../api/client";
import ConnectionError from "../shared/ConnectionError";
import { Input } from "../ui/input";
import { Button } from "../ui/button";
import { useEpisodes } from "../../context/EpisodeContext";

interface ReasoningTracesProps {
  traces?: TraceResponse[];
  task?: string;
  isRunning?: boolean;
  isConnected?: boolean;
}

export default function ReasoningTraces({
  traces,
  task,
  isRunning,
  isConnected = true
}: ReasoningTracesProps) {
  const scrollRef = useRef<HTMLDivElement>(null);
  const { isCreationMode, startAgent, interruptAgent, selectedEpisode } = useEpisodes();
  const [prompt, setPrompt] = useState("");
  const [feedbackState, setFeedbackState] = useState<Record<number, { score: number; comment: string; isSubmitted: boolean }>>({});

  const handleFeedback = async (traceId: number, score: number) => {
    setFeedbackState(prev => ({
      ...prev,
      [traceId]: { ...prev[traceId], score, isSubmitted: false }
    }));
  };

  const submitFeedback = async (traceId: number) => {
    const state = feedbackState[traceId];
    if (!state || !selectedEpisode) return;

    try {
      await submitTraceFeedback(selectedEpisode.id, traceId, state.score, state.comment);
      setFeedbackState(prev => ({
        ...prev,
        [traceId]: { ...prev[traceId], isSubmitted: true }
      }));
    } catch (error) {
      console.error("Failed to submit feedback", error);
    }
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (prompt.trim()) {
      startAgent(prompt);
      setPrompt("");
    }
  };

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
    <div className="flex flex-col h-full bg-card/30 border-r border-border relative">
      {!isConnected && <ConnectionError className="absolute inset-0" />}
      
      {/* Header with Stop Button */}
      <div className="flex items-center justify-between p-2 border-b border-border/50 bg-card/50 backdrop-blur-sm shrink-0">
          <span className="text-[10px] font-bold uppercase tracking-widest opacity-50 pl-2">Reasoning Trace</span>
          {isRunning && selectedEpisode && (
              <Button 
                  variant="destructive" 
                  size="sm" 
                  className="h-6 text-[10px] px-2 uppercase tracking-wider font-bold hover:bg-red-500/20 hover:text-red-400 bg-red-500/10 text-red-500 border border-red-500/20"
                  onClick={() => interruptAgent(selectedEpisode.id)}
              >
                  <Square className="h-3 w-3 mr-1 fill-current" /> Stop
              </Button>
          )}
      </div>

      {/* Creation Mode Prompt */}
      {isCreationMode && !isRunning && (
        <div className="p-4 border-b bg-primary/5">
            <form onSubmit={handleSubmit} className="space-y-3">
                <label className="text-[10px] font-black uppercase tracking-widest text-primary/70">Create Benchmark</label>
                <div className="flex gap-2">
                    <Input 
                        placeholder="Describe the benchmark goal..." 
                        className="h-9 text-xs"
                        value={prompt}
                        onChange={(e) => setPrompt(e.target.value)}
                    />
                    <Button type="submit" size="icon" className="h-9 w-9 shrink-0">
                        <Send className="h-4 w-4" />
                    </Button>
                </div>
            </form>
        </div>
      )}

      {/* Content Area */}
      <div className="flex-1 overflow-hidden flex flex-col relative">
        <ScrollArea className="flex-1 bg-black/20" ref={scrollRef}>
            <div className="p-4 font-mono text-[11px] leading-relaxed">
                {/* Initial Task Message */}
                {task && (
                    <div className="mb-6 p-3 bg-primary/10 rounded-lg border border-primary/20 shadow-sm relative overflow-hidden group">
                        <div className="absolute top-0 right-0 p-1 opacity-20 group-hover:opacity-40 transition-opacity">
                            <Send className="h-3 w-3 text-primary" />
                        </div>
                        <div className="text-[9px] font-black uppercase tracking-widest text-primary/70 mb-1">
                            Initial Objective
                        </div>
                        <div className="text-foreground font-medium leading-relaxed whitespace-pre-wrap">
                            {task}
                        </div>
                    </div>
                )}

                {traces && traces.length > 0 ? (
                    traces.map(trace => (
                        <div key={trace.id} className="space-y-1 mb-3 border-b border-white/5 pb-2 last:border-0 hover:bg-white/5 p-1 rounded transition-colors group">
                            <div className="flex justify-between text-[9px] text-muted-foreground">
                                <span className="text-primary/70 font-bold">[{new Date(trace.created_at).toLocaleTimeString()}]</span>
                                <span className="opacity-50 uppercase group-hover:opacity-100 transition-opacity">Trace: {trace.id}</span>
                            </div>
                            <div className="text-muted-foreground break-all opacity-90 whitespace-pre-wrap">
                                {(() => {
                                    if (trace.trace_type === 'tool_start') {
                                        return (
                                            <div className="text-blue-400">
                                                <span className="font-bold">Tool Call: </span>
                                                <span className="text-blue-300">{trace.name}</span>
                                                <div className="mt-1 bg-blue-900/20 p-2 rounded text-[10px] text-blue-200/70 border border-blue-500/10">
                                                    {trace.content}
                                                </div>
                                            </div>
                                        );
                                    }
                                    if (trace.trace_type === 'tool_end') {
                                        return (
                                            <div className="text-green-400/80">
                                                <span className="font-bold">Output: </span>
                                                <div className="mt-1 bg-green-900/10 p-2 rounded text-[10px] text-green-200/60 border border-green-500/5">
                                                    {trace.content}
                                                </div>
                                            </div>
                                        );
                                    }
                                    if (trace.trace_type === 'llm_end') {
                                        return (
                                            <div className="text-purple-300/90 italic">
                                                {trace.content}
                                            </div>
                                        );
                                    }
                                    
                                    if (trace.trace_type === 'log') {
                                        return <div className="text-muted-foreground">{trace.content}</div>;
                                    }

                                    return trace.content;
                                })()}
                            </div>
                            
                            {/* Feedback UI */}
                            {trace.langfuse_trace_id && (
                                <div className="mt-2 flex flex-col gap-2 p-2 bg-white/5 rounded-md border border-white/5 opacity-0 group-hover:opacity-100 transition-opacity">
                                    <div className="flex items-center gap-4">
                                        <span className="text-[9px] uppercase font-bold tracking-tighter opacity-50">Was this step correct?</span>
                                        <div className="flex gap-2">
                                            <Button 
                                                variant="ghost" 
                                                size="icon" 
                                                className={`h-6 w-6 rounded-full hover:bg-green-500/20 ${feedbackState[trace.id]?.score === 1 ? 'text-green-400 bg-green-500/10' : 'text-muted-foreground'}`}
                                                onClick={() => handleFeedback(trace.id, 1)}
                                                disabled={feedbackState[trace.id]?.isSubmitted}
                                            >
                                                <ThumbsUp className="h-3 w-3" />
                                            </Button>
                                            <Button 
                                                variant="ghost" 
                                                size="icon" 
                                                className={`h-6 w-6 rounded-full hover:bg-red-500/20 ${feedbackState[trace.id]?.score === 0 ? 'text-red-400 bg-red-500/10' : 'text-muted-foreground'}`}
                                                onClick={() => handleFeedback(trace.id, 0)}
                                                disabled={feedbackState[trace.id]?.isSubmitted}
                                            >
                                                <ThumbsDown className="h-3 w-3" />
                                            </Button>
                                        </div>
                                    </div>
                                    
                                    {feedbackState[trace.id]?.score !== undefined && !feedbackState[trace.id]?.isSubmitted && (
                                        <div className="flex gap-2 animate-in fade-in slide-in-from-top-1">
                                            <Input 
                                                placeholder="Provide feedback..." 
                                                className="h-7 text-[10px] bg-black/20"
                                                value={feedbackState[trace.id]?.comment || ""}
                                                onChange={(e) => setFeedbackState(prev => ({
                                                    ...prev,
                                                    [trace.id]: { ...prev[trace.id], comment: e.target.value }
                                                }))}
                                            />
                                            <Button 
                                                size="sm" 
                                                className="h-7 text-[9px] px-3 font-bold uppercase"
                                                onClick={() => submitFeedback(trace.id)}
                                            >
                                                Submit
                                            </Button>
                                        </div>
                                    )}

                                    {feedbackState[trace.id]?.isSubmitted && (
                                        <div className="text-[9px] text-green-400 flex items-center gap-1 font-bold italic animate-in zoom-in-95">
                                            <Check className="h-3 w-3" /> Feedback sent to Langfuse
                                        </div>
                                    )}
                                </div>
                            )}
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
