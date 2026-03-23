import { useState, useRef, useEffect, useCallback, useMemo } from 'react';
import { ScrollArea } from "../../components/ui/scroll-area";
import type { TraceResponse } from "../../api/generated/models/TraceResponse";
import { 
  Check, 
  AlertCircle,
  Zap,
  Play,
  Rocket,
  RotateCcw
} from "lucide-react";
import { runSimulation, type BenchmarkObjectives } from "../../api/client";
import { ObjectivesForm } from "./ObjectivesForm";
import ConnectionError from "../shared/ConnectionError";
import { Button } from "../ui/button";
import { ContextCards } from "./ContextCards";
import { useEpisodes } from "../../context/EpisodeContext";
import { useTheme } from "../../context/ThemeContext";
import { useUISettings } from "../../context/UISettingsContext";
import type { TopologyNode } from "../visualization/ModelBrowser";
import { ChatInput } from "../Chat/ChatInput";
import { TraceList } from "./TraceList";
import { EpisodeStatus } from "../../api/generated/models/EpisodeStatus";
import { EpisodeType } from "../../api/generated/models/EpisodeType";

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
  const { viewReasoning, setViewReasoning } = useUISettings();

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

  const isPlanned =
    selectedEpisode?.status === EpisodeStatus.PLANNED ||
    selectedEpisode?.metadata_vars?.detailed_status === "PLANNED";
  const showExecutionPlan =
    !!selectedEpisode &&
    (isPlanned || !!selectedEpisode.plan) &&
    selectedEpisode.status !== EpisodeStatus.COMPLETED &&
    selectedEpisode.status !== EpisodeStatus.FAILED;
  const reasoningRequired = !!selectedEpisode?.metadata_vars?.additional_info?.reasoning_required;
  const contextUsage = useMemo(() => {
    const usage = selectedEpisode?.metadata_vars?.additional_info?.context_usage as
      | Record<string, unknown>
      | undefined;
    if (!usage) return null;
    const used = Number(usage.used_tokens);
    const max = Number(usage.max_tokens);
    if (!Number.isFinite(used) || !Number.isFinite(max) || max <= 0) {
      return null;
    }
    return { used, max, pct: (used / max) * 100 };
  }, [selectedEpisode?.metadata_vars]);
  const isBenchmarkEpisode =
    selectedEpisode?.metadata_vars?.episode_type === EpisodeType.BENCHMARK ||
    isBenchmarkPath;
  const isEngineerEpisode =
    selectedEpisode?.metadata_vars?.episode_type === EpisodeType.ENGINEER ||
    (!isBenchmarkEpisode && !!selectedEpisode);
  const isFailedEngineerEpisode =
    !!selectedEpisode &&
    selectedEpisode.status === EpisodeStatus.FAILED &&
    selectedEpisode.metadata_vars?.episode_type === EpisodeType.ENGINEER &&
    !!selectedEpisode.metadata_vars?.benchmark_id;
  const terminalMetadata = selectedEpisode?.metadata_vars ?? null;
  const terminalDetailedStatus =
    terminalMetadata?.detailed_status ?? selectedEpisode?.status ?? null;
  const hasStructuredTerminalMetadata =
    !!terminalMetadata?.detailed_status &&
    !!terminalMetadata?.terminal_reason &&
    (selectedEpisode?.status !== EpisodeStatus.FAILED ||
      !!terminalMetadata?.failure_class);
  const shouldShowFallbackLogs =
    selectedEpisode?.status === EpisodeStatus.FAILED &&
    !hasStructuredTerminalMetadata;
  const terminalBannerClass =
    selectedEpisode?.status === EpisodeStatus.FAILED
      ? "mt-6 p-3 bg-red-500/10 rounded-lg border border-red-500/20 shadow-sm"
      : "mt-6 p-3 bg-emerald-500/10 rounded-lg border border-emerald-500/20 shadow-sm";
  const terminalIconClass =
    selectedEpisode?.status === EpisodeStatus.FAILED
      ? "text-red-500"
      : "text-emerald-500";
  const terminalHeading =
    selectedEpisode?.status === EpisodeStatus.FAILED
      ? "Terminal failure"
      : "Terminal outcome";
  const handleRetryFailedEpisode = useCallback(async () => {
    if (!selectedEpisode || !isFailedEngineerEpisode) {
      return;
    }

    const benchmarkId = selectedEpisode.metadata_vars?.benchmark_id?.trim();
    if (!benchmarkId) {
      return;
    }

    await startAgent(
      selectedEpisode.task,
      undefined,
      {
        benchmark_id: benchmarkId,
        prior_episode_id: selectedEpisode.id,
        is_reused: true,
      },
    );
  }, [isFailedEngineerEpisode, selectedEpisode, startAgent]);

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
          <div className="flex items-center gap-3">
            <Zap className="h-4 w-4 text-primary" />
            <span className="text-[11px] font-bold tracking-tight opacity-70">Engineer Workspace</span>
            {contextUsage && (
              <span
                data-testid="context-usage-indicator"
                className="text-[10px] font-mono px-2 py-1 rounded-md border border-border bg-muted/30 text-muted-foreground"
                title="Current context usage"
              >
                Ctx {Math.round(contextUsage.used).toLocaleString()} / {Math.round(contextUsage.max).toLocaleString()} ({contextUsage.pct.toFixed(1)}%)
              </span>
            )}
          </div>
          <button
            data-testid="view-reasoning-toggle"
            type="button"
            onClick={() => setViewReasoning(!viewReasoning)}
            className="text-[10px] font-semibold px-2 py-1 rounded-md border border-border hover:bg-muted transition-colors"
          >
            View reasoning: {viewReasoning ? "On" : "Off"}
          </button>
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
                    showReasoning={viewReasoning}
                    reasoningRequired={reasoningRequired}
                    episodeStatus={selectedEpisode?.status}
                    onAssetClick={handleAssetClick}
                    addToContext={addToContext}
                    onShowFeedback={handleShowFeedback}
                  />
                </div>

                {/* Execution Plan Card (Inline) */}
                {showExecutionPlan && (
                    <div key={`exec-plan-${selectedEpisode?.id}-${selectedEpisode?.status}`} className="mt-8 mb-4 p-4 rounded-xl border-2 border-primary/30 bg-primary/5 shadow-2xl">
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
                                    data-testid="chat-confirm-button"
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

                {isFailedEngineerEpisode && (
                    <div
                      data-testid="failure-summary-container"
                      className="mt-6 p-4 rounded-xl border border-red-500/20 bg-red-500/10 shadow-sm"
                    >
                        <div className="flex items-center gap-2 mb-3 text-red-500">
                            <AlertCircle className="h-4 w-4" />
                            <span className="text-[9px] font-black uppercase tracking-widest">
                                Failed engineer episode
                            </span>
                        </div>
                        <div className="grid grid-cols-1 gap-2 text-[11px] font-mono text-red-400">
                            <div>
                                <span className="font-semibold uppercase tracking-widest text-[10px] opacity-70">Benchmark:</span>{' '}
                                <span data-testid="failure-summary-benchmark-id">
                                  {terminalMetadata?.benchmark_id ?? "unavailable"}
                                </span>
                            </div>
                            <div>
                                <span className="font-semibold uppercase tracking-widest text-[10px] opacity-70">Parent episode:</span>{' '}
                                <span data-testid="failure-summary-prior-episode-id">
                                  {terminalMetadata?.prior_episode_id ?? "root"}
                                </span>
                            </div>
                            <div>
                                <span className="font-semibold uppercase tracking-widest text-[10px] opacity-70">Reused:</span>{' '}
                                <span data-testid="failure-summary-is-reused">
                                  {terminalMetadata?.is_reused ? "true" : "false"}
                                </span>
                            </div>
                        </div>
                        <div className="mt-3 flex flex-wrap items-center gap-2 text-[10px] text-red-400">
                            <span>
                                Terminal reason:{" "}
                                <span data-testid="failure-summary-terminal-reason">
                                  {terminalMetadata?.terminal_reason ?? "unavailable"}
                                </span>
                            </span>
                            <span>
                                Failure class:{" "}
                                <span data-testid="failure-summary-failure-class">
                                  {terminalMetadata?.failure_class ?? "none"}
                                </span>
                            </span>
                        </div>
                        <div className="mt-4 flex items-center gap-2">
                            <Button
                              type="button"
                              data-testid="retry-failed-episode-button"
                              className="bg-red-500/90 hover:bg-red-500 text-white font-black text-[10px] uppercase tracking-widest h-9 shadow-lg shadow-red-500/20"
                              onClick={() => void handleRetryFailedEpisode()}
                            >
                                <RotateCcw className="h-3 w-3 mr-2" />
                                Revise & Retry
                            </Button>
                            <span className="text-[10px] text-red-400/80">
                                Launches a fresh engineer episode against the same benchmark package.
                            </span>
                        </div>
                    </div>
                )}

                {/* Terminal outcome summary */}
                {selectedEpisode && isEngineerEpisode && (
                    <div
                      data-testid="terminal-summary-block"
                      className={terminalBannerClass}
                    >
                        <div className={`flex items-center gap-2 mb-2 ${terminalIconClass}`}>
                            {selectedEpisode.status === EpisodeStatus.FAILED ? (
                                <AlertCircle className="h-4 w-4" />
                            ) : (
                                <Check className="h-4 w-4" />
                            )}
                            <span className="text-[9px] font-black uppercase tracking-widest">
                                {terminalHeading}
                            </span>
                        </div>
                        <div
                          data-testid="terminal-summary-fields"
                          className="grid grid-cols-1 gap-2 text-[11px] font-mono"
                        >
                            <div className={selectedEpisode.status === EpisodeStatus.FAILED ? "text-red-400" : "text-emerald-700 dark:text-emerald-300"}>
                                <span className="font-semibold uppercase tracking-widest text-[10px] opacity-70">Detailed status:</span>{' '}
                                <span data-testid="terminal-summary-detailed-status">
                                  {terminalDetailedStatus ?? "unavailable"}
                                </span>
                            </div>
                            <div className={selectedEpisode.status === EpisodeStatus.FAILED ? "text-red-400" : "text-emerald-700 dark:text-emerald-300"}>
                                <span className="font-semibold uppercase tracking-widest text-[10px] opacity-70">Terminal reason:</span>{' '}
                                <span data-testid="terminal-summary-terminal-reason">
                                  {terminalMetadata?.terminal_reason ?? "unavailable"}
                                </span>
                            </div>
                            <div className={selectedEpisode.status === EpisodeStatus.FAILED ? "text-red-400" : "text-emerald-700 dark:text-emerald-300"}>
                                <span className="font-semibold uppercase tracking-widest text-[10px] opacity-70">Failure class:</span>{' '}
                                <span data-testid="terminal-summary-failure-class">
                                  {terminalMetadata?.failure_class ?? "none"}
                                </span>
                            </div>
                        </div>
                        {shouldShowFallbackLogs && (
                            <div
                              data-testid="terminal-summary-fallback"
                              className="mt-3 text-[11px] text-red-400 font-mono whitespace-pre-wrap"
                            >
                                {selectedEpisode.validation_logs && selectedEpisode.validation_logs.length > 0
                                  ? selectedEpisode.validation_logs.join('\n')
                                  : "The agent encountered an unrecoverable error during execution."}
                            </div>
                        )}
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
