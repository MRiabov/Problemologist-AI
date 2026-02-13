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
import { vscDarkPlus, vs } from "react-syntax-highlighter/dist/esm/styles/prism";
import { submitTraceFeedback, runSimulation } from "../../api/client";
import ConnectionError from "../shared/ConnectionError";
import { Input } from "../ui/input";
import { Button } from "../ui/button";
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

import { getFileIconInfo } from "../../lib/fileIcons";

const ActionCard = ({ trace }: { trace: TraceResponse }) => {
    const { setActiveArtifactId, selectedEpisode } = useEpisodes();
    const isStart = trace.trace_type === 'tool_start';
    const isEnd = trace.trace_type === 'tool_end';
    
    if (!isStart && !isEnd) return null;

    // Determine tool name and arguments
    const toolName = trace.name || "";
    let args: any = {};
    if (trace.content) {
        try {
            args = JSON.parse(trace.content);
        } catch (e) {
            // Content might be a string or other data
        }
    }

    const isFileTool = ['write_file', 'read_file', 'replace_file_content', 'multi_replace_file_content'].includes(toolName);
    const filePath = args.TargetFile || args.AbsolutePath || args.path || "";
    const fileName = filePath.split('/').pop() || filePath;

    const handleActionClick = () => {
        if (!isFileTool || !filePath || !selectedEpisode) return;
        
        // Try to find matching asset ID
        const asset = selectedEpisode.assets?.find(a => a.s3_path.endsWith(filePath) || a.s3_path === filePath);
        if (asset) {
            setActiveArtifactId(asset.id.toString());
        } else if (filePath.toLowerCase().endsWith('plan.md')) {
            setActiveArtifactId('plan');
        }
    };

    const getIcon = () => {
        if (isFileTool && fileName) {
            const { icon: FileIcon, color } = getFileIconInfo(fileName);
            return <FileIcon className="h-3.5 w-3.5" style={{ color }} />;
        }

        const name = toolName.toLowerCase();
        if (name.includes('write')) return <FileEdit className="h-3.5 w-3.5 text-blue-400" />;
        if (name.includes('read')) return <Eye className="h-3.5 w-3.5 text-emerald-400" />;
        if (name.includes('list') || name.includes('ls')) return <Folder className="h-3.5 w-3.5 text-amber-400" />;
        if (name.includes('run') || name.includes('exec') || name.includes('command')) return <PlayCircle className="h-3.5 w-3.5 text-purple-400" />;
        if (name.includes('search') || name.includes('cots')) return <Search className="h-3.5 w-3.5 text-sky-400" />;
        return <Zap className="h-3.5 w-3.5 text-gray-400" />;
    };

    const getLabel = () => {
        const name = toolName.toLowerCase();
        if (name.includes('write')) return 'Edited';
        if (name.includes('read')) return 'Read';
        if (name.includes('list')) return 'Viewed';
        if (name.includes('run')) return 'Ran';
        if (name.includes('search')) return 'COTS';
        return 'Tool';
    };

    // For file tools, we show a simplified one-line view
    if (isStart && isFileTool && fileName) {
        return (
            <div 
                onClick={handleActionClick}
                className={cn(
                    "group flex items-center gap-2 p-1.5 px-2 rounded-md border border-border/50 transition-all mb-1 cursor-pointer",
                    "bg-muted/30 hover:bg-muted/50 hover:border-primary/30"
                )}
            >
                <div className="flex items-center gap-2 flex-1 min-w-0">
                    <span className="text-[10px] font-bold uppercase tracking-wider text-muted-foreground whitespace-nowrap">
                        {getLabel()}:
                    </span>
                    <div className="flex items-center gap-1.5 overflow-hidden">
                        {getIcon()}
                        <span className="text-[11px] font-mono text-foreground truncate">
                            {fileName}
                        </span>
                    </div>
                </div>
                <div className="flex items-center gap-1 shrink-0">
                    <span className="text-[10px] text-green-500 font-bold opacity-70">+</span>
                    <span className="text-[10px] text-red-500 font-bold opacity-70">-</span>
                </div>
            </div>
        );
    }

    // Default ActionCard for other tools or end signals
    return (
        <div className={cn(
            "group flex flex-col gap-1 p-2 rounded-md border border-border/50 transition-all mb-2",
            isStart ? "bg-muted/30 hover:bg-muted/50" : "bg-muted/10"
        )}>
            <div className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                    {getIcon()}
                    <span className="text-[10px] font-bold uppercase tracking-wider text-muted-foreground">
                        {getLabel()}: <span className="text-foreground">{toolName}</span>
                    </span>
                </div>
                <span className="text-[9px] opacity-30 font-mono">#{trace.id}</span>
            </div>
            {isStart && trace.content && !isFileTool && (
                <div className="mt-1 overflow-hidden rounded border border-border/20">
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
            <span className="text-[10px] font-black uppercase tracking-widest opacity-70">Chat Window</span>
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
                                  <div className="text-muted-foreground break-words opacity-90 whitespace-pre-wrap text-[11px] leading-relaxed">
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
            </div>
        </ScrollArea>

        {/* Steering / Input Area */}
        <div className="p-4 border-t border-border/50 bg-card/50 backdrop-blur-sm">
            <form onSubmit={handleSubmit} className="relative group">
                <Input 
                   id="chat-input"
                   placeholder={isRunning ? "Steer the agent..." : "Start a new task..."}
                   className="h-12 pl-4 pr-12 bg-muted/20 border-border/50 focus:border-primary/50 focus:ring-primary/20 transition-all text-xs"
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
