import { useRef, useEffect, useState } from "react";
import { ScrollArea } from "../../components/ui/scroll-area";
import type { TraceResponse } from "../../api/generated/models/TraceResponse";
import { 
  Terminal, 
  Check, 
  AlertCircle,
  Zap,
  Play,
  ChevronUp,
  ChevronDown,
  Rocket,
  Plus,
  Mic,
  ArrowRight,
  Square
} from "lucide-react";
import { ThoughtBlock } from "./ThoughtBlock";
import { ActionCard } from "./ActionCard";
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { vscDarkPlus, vs } from "react-syntax-highlighter/dist/esm/styles/prism";
import { runSimulation, type BenchmarkObjectives } from "../../api/client";
import { ObjectivesForm } from "./ObjectivesForm";
import ConnectionError from "../shared/ConnectionError";
import { Button } from "../ui/button";
import { ContextCards } from "./ContextCards";
import { FeedbackSystem } from "./FeedbackSystem";
import { useEpisodes } from "../../context/EpisodeContext";
import { useTheme } from "../../context/ThemeContext";
import { cn } from "../../lib/utils";

// Internal helper for rendering highlighted content
const HighlightedContent = ({ content, language = 'text' }: { content: string, language?: string }) => {
    const { theme } = useTheme();
    if (!content) return null;
    // Basic detection for JSON strings
    const lang = (content.trim().startsWith('{') || content.trim().startsWith('[')) ? 'json' : language;
    
    return (
        <SyntaxHighlighter
            language={lang}
            style={theme === 'dark' ? vscDarkPlus : vs}
            customStyle={{
                margin: 0,
                padding: '0.5rem',
                background: theme === 'dark' ? 'rgba(0,0,0,0.2)' : 'rgba(0,0,0,0.05)',
                fontSize: '10px',
                borderRadius: '4px',
                border: theme === 'dark' ? '1px solid rgba(255,255,255,0.05)' : '1px solid rgba(0,0,0,0.05)'
            }}
            wrapLines={true}
            wrapLongLines={true}
        >
            {content}
        </SyntaxHighlighter>
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
  const { 
      isCreationMode, 
      startAgent, 
      confirmBenchmark,
      interruptAgent, 
      selectedEpisode, 
      updateObjectives, 
      episodes,
      selectedContext,
      clearContext
  } = useEpisodes();
  const [prompt, setPrompt] = useState("");
  const [selectedBenchmarkId, setSelectedBenchmarkId] = useState<string>("");

  const [showObjectives, setShowObjectives] = useState(false);
  const [showFeedbackModal, setShowFeedbackModal] = useState(false);
  const [objectives, setObjectives] = useState<BenchmarkObjectives>({});

  const location = window.location;
  const isBenchmarkPath = location.pathname === '/benchmark';


  // Reset objectives when episode changes (optional, or fetch from episode metadata)
  useEffect(() => {
      if (selectedEpisode?.metadata_vars?.custom_objectives) {
          setObjectives(selectedEpisode.metadata_vars.custom_objectives as BenchmarkObjectives);
      } else if (isCreationMode) {
          setObjectives({});
      }
  }, [selectedEpisode, isCreationMode]);




  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (prompt.trim()) {
      const metadata = {
        ...(selectedBenchmarkId ? { benchmark_id: selectedBenchmarkId } : {}),
        ...(selectedContext.length > 0 ? { context_items: selectedContext } : {})
      };
      
      startAgent(prompt, objectives, Object.keys(metadata).length > 0 ? metadata : undefined);
      
      setPrompt("");
      setSelectedBenchmarkId("");
      clearContext();
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
  const isPlanned = selectedEpisode?.metadata_vars?.detailed_status === 'planned';
  const showExecutionPlan = (selectedEpisode?.plan || isPlanned) && !isRunning && selectedEpisode?.status !== 'completed';

  return (
    <div className="flex flex-col h-full bg-background relative overflow-hidden">
      {!isConnected && <ConnectionError className="absolute inset-0 z-50" />}
      
      {/* Header */}
      <div className="flex items-center justify-between p-3 border-b border-white/5 bg-background shrink-0">
          <div className="flex items-center gap-2">
            <Zap className="h-4 w-4 text-primary" />
            <span className="text-[11px] font-bold tracking-tight opacity-70">Engineer Workspace</span>
          </div>
      </div>

      {/* Main Content Area */}
      <div className="flex-1 overflow-hidden flex flex-col relative">
        <ScrollArea className="flex-1" ref={scrollRef}>
            <div className="p-4 flex flex-col min-h-full">
                {/* Creation Mode Prompt */}
                {isCreationMode && !task && (
                    <div className="mb-6 p-4 bg-primary/5 rounded-xl border-2 border-dashed border-primary/20 animate-in fade-in zoom-in-95 duration-500">
                        <div className="flex flex-col items-center text-center gap-3">
                            <div className="size-10 flex items-center justify-center bg-primary/20 rounded-full text-primary">
                                {isBenchmarkPath ? <Rocket className="h-5 w-5" /> : <Zap className="h-5 w-5" />}
                            </div>
                            <div className="space-y-1">
                                <h3 className="text-xs font-black uppercase tracking-widest text-primary">
                                    {isBenchmarkPath ? 'Benchmark Creation Mode' : 'New Design Session'}
                                </h3>
                                <p className="text-[10px] text-muted-foreground leading-relaxed max-w-[200px]">
                                    {isBenchmarkPath 
                                        ? 'Enter a high-level prompt below to begin generating a new benchmark plan.'
                                        : 'Describe the part or mechanism you want to design to start a new session.'
                                    }
                                </p>
                            </div>
                        </div>
                    </div>
                )}

                {/* Initial Task Message (User Style) */}
                {task && (
                    <div className="flex justify-end mb-8 animate-in slide-in-from-right-4 fade-in duration-300">
                        <div className="max-w-[85%] bg-muted/40 rounded-2xl p-4 shadow-sm border border-border/10 relative group">
                            <div className="text-[13px] leading-relaxed text-foreground whitespace-pre-wrap">
                                {task}
                            </div>
                            <button className="absolute bottom-2 right-4 opacity-0 group-hover:opacity-40 transition-opacity">
                                <Rocket className="h-3.5 w-3.5" />
                            </button>
                        </div>
                    </div>
                )}

                {/* Messages / Traces */}
                <div className="space-y-4 max-w-3xl mx-auto w-full">
                  {traces && traces.length > 0 ? (
                      traces.map(trace => {
                          const type = trace.trace_type as string;
                          if (type === 'llm_thought' || type === 'thought') {
                            return <ThoughtBlock key={trace.id} duration={Math.floor(Math.random() * 5) + 1} content={trace.content || ""} />;
                          }
                          if (type === 'tool_start') {
                              return <ActionCard key={trace.id} trace={trace} />;
                          }
                          if (type === 'llm_end' && trace.content) {
                            return (
                                <div key={trace.id} className="relative group/msg">
                                    <div className="text-[13px] leading-relaxed text-foreground/90 whitespace-pre-wrap py-1">
                                         <HighlightedContent content={trace.content} language="markdown" />
                                    </div>
                                    <div className="flex items-center gap-2 mt-1 opacity-0 group-hover/msg:opacity-100 transition-opacity">
                                        <button 
                                            onClick={() => setShowFeedbackModal(true)}
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
                      })
                  ) : (
                      <div className="flex flex-col items-center justify-center py-20 gap-2 opacity-10 h-full">
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
                                        if (!selectedEpisode) return;
                                        try {
                                            if (isPlanned) {
                                                await confirmBenchmark(selectedEpisode.id);
                                            } else {
                                                const sessionId = `sim-${Math.random().toString(36).substring(2, 10)}`;
                                                await runSimulation(sessionId);
                                            }
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
                                    className="px-3 border-border/50 hover:bg-muted/50 text-[10px] uppercase font-bold text-muted-foreground"
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
                {/* Session Feedback Modal */}
                {showFeedbackModal && selectedEpisode && (
                    <FeedbackSystem 
                        episodeId={selectedEpisode.id} 
                        onClose={() => setShowFeedbackModal(false)}
                    />
                )}
            </div>
        </ScrollArea>

        {/* Steering / Input Area (Claude Inspired) */}
        <div className="p-4 max-w-3xl mx-auto w-full relative">
            <ContextCards />
            
            <div className="bg-muted/30 border border-white/5 rounded-2xl p-3 shadow-sm focus-within:ring-1 focus-within:ring-primary/20 transition-all">
                <form onSubmit={handleSubmit} className="flex flex-col gap-2">
                    <textarea 
                       id="chat-input"
                       placeholder={isRunning ? "Steer the agent..." : "Ask anything (Ctrl+L), @ to mention, / for workflows"}
                       className="w-full bg-transparent border-none focus:ring-0 text-[14px] leading-relaxed resize-none min-h-[40px] max-h-48"
                       rows={1}
                       value={prompt}
                       onChange={(e) => {
                           setPrompt(e.target.value);
                           e.target.style.height = 'auto';
                           e.target.style.height = e.target.scrollHeight + 'px';
                       }}
                       onKeyDown={(e) => {
                           if (e.key === 'Enter' && !e.shiftKey) {
                               e.preventDefault();
                               handleSubmit(e);
                           }
                       }}
                    />
                    
                    <div className="flex items-center justify-between mt-1">
                        <div className="flex items-center gap-1.5">
                            <Button variant="ghost" size="icon" className="h-7 w-7 rounded-lg hover:bg-white/5">
                                <Plus className="h-4 w-4 opacity-40" />
                            </Button>
                            
                            <Button 
                                variant="ghost" 
                                size="sm" 
                                className="h-7 px-2 gap-1 rounded-lg hover:bg-white/5 text-[11px] font-medium text-muted-foreground"
                                onClick={(e) => {
                                    e.preventDefault();
                                    setShowObjectives(!showObjectives);
                                }}
                            >
                                <Zap className="h-3 w-3 opacity-40" />
                                Planning
                                {showObjectives ? <ChevronDown className="h-3 w-3 ml-0.5 opacity-40" /> : <ChevronUp className="h-3 w-3 ml-0.5 opacity-40" />}
                            </Button>

                            <Button 
                                variant="ghost" 
                                size="sm" 
                                className="h-7 px-2 gap-1 rounded-lg hover:bg-white/5 text-[11px] font-medium text-muted-foreground"
                            >
                                <Rocket className="h-3 w-3 opacity-40" />
                                Claude Opus 4.6
                            </Button>
                        </div>

                        <div className="flex items-center gap-1.5">
                            <Button variant="ghost" size="icon" className="h-8 w-8 rounded-lg hover:bg-white/5">
                                <Mic className="h-4 w-4 opacity-40" />
                            </Button>
                            
                            <Button 
                               type={isRunning ? "button" : "submit"}
                               size="icon" 
                               disabled={!isRunning && !prompt.trim()}
                               className={cn(
                                   "h-8 w-8 rounded-full transition-all flex items-center justify-center",
                                   isRunning 
                                    ? "bg-red-500/20 text-red-500 hover:bg-red-500/30" 
                                    : "bg-foreground text-background hover:opacity-90 disabled:bg-muted/20 disabled:text-muted-foreground"
                               )}
                               onClick={(e) => {
                                   if (isRunning && selectedEpisode) {
                                       e.preventDefault();
                                       interruptAgent(selectedEpisode.id);
                                   }
                               }}
                            >
                                {isRunning ? <Square className="h-3.5 w-3.5 fill-current" /> : <ArrowRight className="h-4 w-4" />}
                            </Button>
                        </div>
                    </div>
                </form>
            </div>

            {showObjectives && (
                <div className="absolute bottom-full left-0 right-0 mb-2 px-4 animate-in slide-in-from-bottom-2 duration-200">
                    <div className="bg-background border border-border shadow-2xl rounded-xl overflow-hidden">
                        <ObjectivesForm 
                            objectives={objectives} 
                            onChange={setObjectives} 
                            onUpdate={() => updateObjectives(objectives)}
                            showUpdate={!isCreationMode && !!selectedEpisode}
                            disabled={isRunning}
                        />
                    </div>
                </div>
            )}
        </div>
      </div>
    </div>
  );
}
