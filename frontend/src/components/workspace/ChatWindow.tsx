import { useState, useRef, useEffect, useCallback, useMemo } from 'react';
import { ScrollArea } from "../../components/ui/scroll-area";
import type { TraceResponse } from "../../api/generated/models/TraceResponse";
import { 
  Check, 
  AlertCircle,
  Zap,
  Play,
  Rocket
} from "lucide-react";
import { runSimulation, type BenchmarkObjectives } from "../../api/client";
import { ObjectivesForm } from "./ObjectivesForm";
import ConnectionError from "../shared/ConnectionError";
import { Button } from "../ui/button";
import { ContextCards } from "./ContextCards";
import { useEpisodes } from "../../context/EpisodeContext";
import { useTheme } from "../../context/ThemeContext";
import type { TopologyNode } from "../visualization/ModelBrowser";
import { ChatInput } from "../Chat/ChatInput";
import { TraceList } from "./TraceList";

interface ChatWindowProps {
  traces?: TraceResponse[];
  task?: string;
  isRunning?: boolean;
  isConnected?: boolean;
  topologyNodes?: TopologyNode[];
}

export default function ChatWindow({
  traces,
  task,
  isRunning,
  isConnected = true,
  topologyNodes = []
}: ChatWindowProps) {
  const scrollRef = useRef<HTMLDivElement>(null);
  const { 
      isCreationMode, 
      startAgent, 
      continueAgent,
      steerAgent,
      confirmBenchmark,
      interruptAgent, 
      selectedEpisode, 
      updateObjectives, 
      selectedContext,
      addToContext,
      setActiveArtifactId,
      setFeedbackState
  } = useEpisodes();
  const { theme } = useTheme();

  const [objectives, setObjectives] = useState<BenchmarkObjectives>({});
  const [showObjectives, setShowObjectives] = useState(false);
  const [confirmComment, setConfirmComment] = useState("");

  const location = window.location;
  const isBenchmarkPath = location.pathname === '/benchmark';

  const handleSendMessage = async (prompt: string, metadata: any) => {
    // Add objectives to startAgent if needed
    if (selectedEpisode && !isCreationMode) {
        if (isRunning) {
            await steerAgent(selectedEpisode.id, prompt, metadata);
        } else {
            await continueAgent(selectedEpisode.id, prompt, metadata);
        }
    } else {
        await startAgent(prompt, objectives, Object.keys(metadata).length > 0 ? metadata : undefined);
    }
  };


  // Reset objectives when episode changes
  useEffect(() => {
      if (selectedEpisode?.metadata_vars?.custom_objectives) {
          setObjectives(selectedEpisode.metadata_vars.custom_objectives as BenchmarkObjectives);
      } else if (isCreationMode) {
          setObjectives({});
      }
  }, [selectedEpisode, isCreationMode]);



  // Auto-scroll to bottom when new traces arrive
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

  const isPlanned = selectedEpisode?.metadata_vars?.detailed_status === 'planned';
  const showExecutionPlan = (selectedEpisode?.plan || isPlanned) && !isRunning && selectedEpisode?.status !== 'completed';

  // Stable handlers
  const handleShowFeedback = useCallback((traceId: number, score: number) => {
    setFeedbackState({ traceId, score });
  }, [setFeedbackState]);
  const handleAssetClick = useCallback((id: string | null) => setActiveArtifactId(id), [setActiveArtifactId]);

  // Stable assets list to prevent re-renders on every poll
  const stableAssets = useMemo(() => {
    return selectedEpisode?.assets || [];
    // We use JSON.stringify to ensure deep equality check, as polling returns new object references
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selectedEpisode ? JSON.stringify(selectedEpisode.assets) : '']);

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
                  <TraceList
                    traces={traces}
                    assets={stableAssets}
                    theme={theme}
                    onAssetClick={handleAssetClick}
                    addToContext={addToContext}
                    onShowFeedback={handleShowFeedback}
                    isRunning={isRunning}
                  />
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
                            <div className="w-full space-y-2 mt-2">
                                <textarea
                                    placeholder="Optional comment for the agent..."
                                    value={confirmComment}
                                    onChange={(e) => setConfirmComment(e.target.value)}
                                    className="w-full bg-background/50 border border-primary/20 rounded-lg p-2 text-xs focus:ring-1 focus:ring-primary/30 outline-none resize-none h-16"
                                />
                            </div>

                            <div className="flex gap-2 w-full mt-2">
                                <Button 
                                    className="flex-1 bg-primary hover:bg-primary/90 text-primary-foreground font-black text-[10px] uppercase tracking-widest h-10 shadow-lg shadow-primary/20"
                                    onClick={async () => {
                                        if (!selectedEpisode) return;
                                        try {
                                            if (isPlanned) {
                                                await confirmBenchmark(selectedEpisode.id, confirmComment);
                                            } else {
                                                const sessionId = `sim-${Math.random().toString(36).substring(2, 10)}`;
                                                await runSimulation(sessionId);
                                            }
                                            setConfirmComment("");
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
        <div className="p-4 max-w-3xl mx-auto w-full relative">
            <ContextCards />
            
            <ChatInput 
                onSendMessage={handleSendMessage}
                isRunning={!!isRunning}
                onInterrupt={() => selectedEpisode && interruptAgent(selectedEpisode.id)}
                selectedEpisode={selectedEpisode}
                selectedContext={selectedContext}
                topologyNodes={topologyNodes}
                addToContext={addToContext}
                showObjectives={showObjectives}
                setShowObjectives={setShowObjectives}
            />

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
