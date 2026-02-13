import { useRef, useEffect, useState } from "react";
import { ScrollArea } from "../../components/ui/scroll-area";
import type { TraceResponse } from "../../api/generated/models/TraceResponse";
import { 
  Terminal, 
  Send, 
  Square, 
  ThumbsUp, 
  ThumbsDown, 
  Check, 
  AlertCircle,
  FileEdit,
  Eye,
  Folder,
  PlayCircle,
  Search,
  Zap,
  Clock,
  Play
} from "lucide-react";
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { vscDarkPlus } from "react-syntax-highlighter/dist/esm/styles/prism";
import { submitTraceFeedback, runSimulation } from "../../api/client";
import ConnectionError from "../shared/ConnectionError";
import { Input } from "../ui/input";
import { Button } from "../ui/button";
import { useEpisodes } from "../../context/EpisodeContext";
import { cn } from "../../lib/utils";

// Internal helper for rendering highlighted content
const HighlightedContent = ({ content, language = 'text' }: { content: string, language?: string }) => {
    if (!content) return null;
    // Basic detection for JSON strings
    const lang = (content.trim().startsWith('{') || content.trim().startsWith('[')) ? 'json' : language;
    
    return (
        <SyntaxHighlighter
            language={lang}
            style={vscDarkPlus}
            customStyle={{
                margin: 0,
                padding: '0.5rem',
                background: 'rgba(0,0,0,0.2)',
                fontSize: '10px',
                borderRadius: '4px',
                border: '1px solid rgba(255,255,255,0.05)'
            }}
            wrapLines={true}
            wrapLongLines={true}
        >
            {content}
        </SyntaxHighlighter>
    );
};

const ActionCard = ({ trace }: { trace: TraceResponse }) => {
    const isStart = trace.trace_type === 'tool_start';
    const isEnd = trace.trace_type === 'tool_end';
    
    if (!isStart && !isEnd) return null;

    const getIcon = () => {
        const name = trace.name?.toLowerCase() || "";
        if (name.includes('write')) return <FileEdit className="h-3 w-3 text-blue-400" />;
        if (name.includes('read')) return <Eye className="h-3 w-3 text-emerald-400" />;
        if (name.includes('list') || name.includes('ls')) return <Folder className="h-3 w-3 text-amber-400" />;
        if (name.includes('run') || name.includes('exec') || name.includes('command')) return <PlayCircle className="h-3 w-3 text-purple-400" />;
        if (name.includes('search') || name.includes('cots')) return <Search className="h-3 w-3 text-sky-400" />;
        return <Zap className="h-3 w-3 text-gray-400" />;
    };

    const getLabel = () => {
        const name = trace.name?.toLowerCase() || "";
        const action = isStart ? (name.includes('write') ? 'Edited' : name.includes('read') ? 'Read' : name.includes('list') ? 'Viewed' : name.includes('run') ? 'Ran' : name.includes('search') ? 'COTS' : 'Tool') : 'Output';
        return action;
    };

    return (
        <div className={cn(
            "group flex flex-col gap-1 p-2 rounded-md border border-white/5 transition-all mb-2",
            isStart ? "bg-white/5 hover:bg-white/10" : "bg-black/20"
        )}>
            <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                    {getIcon()}
                    <span className="text-[10px] font-bold uppercase tracking-wider text-muted-foreground">
                        {getLabel()}: <span className="text-foreground">{trace.name}</span>
                    </span>
                </div>
                <span className="text-[9px] opacity-30 font-mono">#{trace.id}</span>
            </div>
            {trace.content && (
                <div className="mt-1 overflow-hidden rounded">
                    <HighlightedContent content={trace.content} />
                </div>
            )}
        </div>
    );
};

interface ChatWindowProps {
  traces?: TraceResponse[];
  task?: string;
  isRunning?: boolean;
  isConnected?: boolean;
}

export default function ChatWindow({
  traces,
  task,
  isRunning,
  isConnected = true
}: ChatWindowProps) {
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

  // Auto-scroll to bottom when new traces arrive, but only if the user is already at the bottom
  useEffect(() => {
    if (traces && traces.length > 0) {
      const timeout = setTimeout(() => {
        const scrollContainer = scrollRef.current?.querySelector('[data-radix-scroll-area-viewport]');
        if (scrollContainer) {
          const { scrollTop, scrollHeight, clientHeight } = scrollContainer;
          const isAtBottom = scrollHeight - scrollTop - clientHeight < 150;
          if (isAtBottom) {
            scrollContainer.scrollTop = scrollContainer.scrollHeight;
          }
        }
      }, 100);
      return () => clearTimeout(timeout);
    }
  }, [traces]);

  // Determine if we should show the Execution Plan card
  // It shows if there's a plan and the agent is not running (waiting for approval)
  const showExecutionPlan = selectedEpisode?.plan && !isRunning && selectedEpisode.status !== 'completed';

  return (
    <div className="flex flex-col h-full bg-card/30 border-r border-border relative overflow-hidden">
      {!isConnected && <ConnectionError className="absolute inset-0 z-50" />}
      
      {/* Header */}
      <div className="flex items-center justify-between p-3 border-b border-border/50 bg-card/50 backdrop-blur-sm shrink-0">
          <div className="flex items-center gap-2">
            <Zap className="h-4 w-4 text-primary" />
            <span className="text-[10px] font-black uppercase tracking-widest opacity-70">Reasoning Window</span>
          </div>
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

      {/* Main Content Area */}
      <div className="flex-1 overflow-hidden flex flex-col relative">
        <ScrollArea className="flex-1" ref={scrollRef}>
            <div className="p-4 flex flex-col min-h-full">
                {/* Initial Task Message */}
                {task && (
                    <div className="mb-6 p-3 bg-primary/10 rounded-lg border border-primary/20 shadow-sm relative overflow-hidden group">
                        <div className="absolute top-0 right-0 p-1 opacity-20 group-hover:opacity-40 transition-opacity">
                            <Send className="h-3 w-3 text-primary" />
                        </div>
                        <div className="text-[9px] font-black uppercase tracking-widest text-primary/70 mb-1">
                            Current Objective
                        </div>
                        <div className="text-foreground font-medium leading-relaxed text-[11px] whitespace-pre-wrap max-h-32 overflow-y-auto pr-1 scrollbar-thin">
                            {task}
                        </div>
                    </div>
                )}

                {/* Messages / Traces */}
                <div className="space-y-4">
                  {traces && traces.length > 0 ? (
                      traces.map(trace => {
                          if (trace.trace_type === 'tool_start' || trace.trace_type === 'tool_end') {
                              return <ActionCard key={trace.id} trace={trace} />;
                          }

                          return (
                              <div key={trace.id} className="space-y-1 group">
                                  <div className="flex justify-between text-[9px] text-muted-foreground opacity-50 group-hover:opacity-100 transition-opacity">
                                      <div className="flex items-center gap-1">
                                          <Clock className="h-2.5 w-2.5" />
                                          <span>{new Date(trace.created_at).toLocaleTimeString()}</span>
                                      </div>
                                      <span className="uppercase">{trace.trace_type}</span>
                                  </div>
                                  <div className="text-muted-foreground break-all opacity-90 whitespace-pre-wrap text-[11px] leading-relaxed">
                                      {trace.trace_type === 'llm_end' ? (
                                          <HighlightedContent content={trace.content} />
                                      ) : (
                                          trace.content
                                      )}
                                  </div>
                              </div>
                          );
                      })
                  ) : (
                      <div className="flex flex-col items-center justify-center py-20 gap-2 opacity-20 h-full">
                         <Terminal className="h-8 w-8" />
                         <span className="text-[10px] uppercase font-bold tracking-widest">Awaiting interaction...</span>
                      </div>
                  )}
                </div>

                {/* Execution Plan Card (Inline) */}
                {showExecutionPlan && (
                    <div className="mt-8 mb-4 p-4 rounded-xl border-2 border-primary/30 bg-primary/5 shadow-2xl animate-in fade-in slide-in-from-bottom-4 duration-500">
                        <div className="flex flex-col items-center text-center gap-3">
                            <div className="size-12 flex items-center justify-center bg-primary/20 rounded-full text-primary ring-4 ring-primary/10">
                                <Check className="h-6 w-6 stroke-[3]" />
                            </div>
                            <div className="space-y-1">
                                <h3 className="text-sm font-black uppercase tracking-tighter text-foreground">Execution Plan Ready</h3>
                                <p className="text-xs text-muted-foreground leading-relaxed max-w-[240px]">
                                    I have drafted the implementation strategy. Please review it in the explorer and confirm to proceed.
                                </p>
                            </div>
                            <div className="flex gap-2 w-full mt-2">
                                <Button 
                                    className="flex-1 bg-primary hover:bg-primary/90 text-primary-foreground font-black text-[10px] uppercase tracking-widest h-10 shadow-lg shadow-primary/20"
                                    onClick={async () => {
                                        try {
                                            const sessionId = `sim-${Math.random().toString(36).substring(2, 10)}`;
                                            await runSimulation(sessionId);
                                        } catch (e) {
                                            console.error("Failed to start implementation", e);
                                        }
                                    }}
                                >
                                    <Play className="h-3 w-3 mr-2 fill-current" />
                                    Confirm & Start
                                </Button>
                                <Button 
                                    variant="outline"
                                    className="px-3 border-white/10 hover:bg-white/5 text-[10px] uppercase font-bold text-muted-foreground"
                                    onClick={() => {
                                      // Logic for requesting changes - could just focus input
                                      const input = document.getElementById('chat-input');
                                      input?.focus();
                                    }}
                                >
                                    Request Changes
                                </Button>
                            </div>
                        </div>
                    </div>
                )}
                
                {/* Visual indicator for active streaming */}
                {isRunning && (
                    <div className="mt-4 flex items-center gap-2 text-primary animate-pulse pl-1 opacity-50 text-[10px] font-mono">
                        <div className="size-1 bg-primary rounded-full" />
                        <span>Agent is thinking...</span>
                    </div>
                )}

                {/* Failure Message */}
                {selectedEpisode?.status === 'failed' && (
                    <div className="mt-6 p-3 bg-red-500/10 rounded-lg border border-red-500/20 shadow-sm">
                        <div className="flex items-center gap-2 mb-2 text-red-500">
                            <AlertCircle className="h-4 w-4" />
                            <span className="text-[9px] font-black uppercase tracking-widest">Terminal failure</span>
                        </div>
                        <div className="text-[11px] text-red-400 font-mono whitespace-pre-wrap">
                            {selectedEpisode.validation_logs && selectedEpisode.validation_logs.length > 0 
                                ? selectedEpisode.validation_logs.join('\n')
                                : "The agent encountered an unrecoverable error during execution."}
                        </div>
                    </div>
                )}
            </div>
        </ScrollArea>

        {/* Steering / Input Area */}
        <div className="p-4 border-t border-border/50 bg-card/50 backdrop-blur-sm">
            <form onSubmit={handleSubmit} className="relative group">
                <Input 
                   id="chat-input"
                   placeholder={isRunning ? "Steer the agent..." : "Start a new task..."}
                   className="h-12 pl-4 pr-12 bg-black/40 border-white/5 focus:border-primary/50 focus:ring-primary/20 transition-all text-xs"
                   value={prompt}
                   onChange={(e) => setPrompt(e.target.value)}
                />
                <Button 
                   type="submit" 
                   size="icon" 
                   disabled={!prompt.trim()}
                   className="absolute right-1 top-1 h-10 w-10 bg-transparent hover:bg-primary/10 text-primary transition-all disabled:opacity-0"
                >
                    <Send className="h-4 w-4" />
                </Button>
            </form>
            <div className="mt-2 flex items-center justify-between px-1">
                <span className="text-[9px] text-muted-foreground/40 font-bold uppercase tracking-widest">
                    {isRunning ? 'Steering Mode Active' : 'Ready for target'}
                </span>
                <div className="flex items-center gap-1.5 opacity-20">
                   <div className="size-1.5 rounded-full bg-green-500 animate-pulse" />
                   <span className="text-[8px] font-bold uppercase">System Link</span>
                </div>
            </div>
        </div>
      </div>
    </div>
  );
}
