import { NavLink, useLocation, useNavigate } from "react-router-dom";
import { LayoutDashboard, Rocket, Settings, History, CheckCircle2, XCircle, Clock, Search, Layers, Plus, ThumbsUp, ThumbsDown } from "lucide-react";
import { cn } from "../../lib/utils";
import { useEpisodes } from "../../context/EpisodeContext";
import { ScrollArea } from "../ui/scroll-area";
import { Input } from "../ui/input";
import { Button } from "../ui/button";
import { useState, useCallback } from "react";
import { EpisodeStatus } from "../../api/generated/models/EpisodeStatus";
import { EpisodePhase } from "../../api/generated/models/EpisodePhase";
import RevisionLineageSummary from "../workspace/RevisionLineageSummary";

const navigation = [
  { name: "Workspace", href: "/", icon: LayoutDashboard },
  { name: "Benchmark", href: "/benchmark", icon: Rocket },
];

const phaseProgressByEpisodePhase: Partial<Record<EpisodePhase, number>> = {
  [EpisodePhase.BENCHMARK_PLANNING]: 20,
  [EpisodePhase.BENCHMARK_PLAN_REVIEWING]: 35,
  [EpisodePhase.BENCHMARK_CODING]: 60,
  [EpisodePhase.BENCHMARK_REVIEWING]: 85,
  [EpisodePhase.ENGINEERING_PLANNING]: 20,
  [EpisodePhase.ENGINEERING_CODING]: 60,
  [EpisodePhase.ENGINEERING_REVIEWING]: 85,
};

function getEpisodeRailProgress(ep: {
  status: EpisodeStatus;
  metadata_vars?: {
    detailed_status?: string | null;
    episode_phase?: EpisodePhase | null;
  } | null;
}): number | null {
  const detailedStatus = ep.metadata_vars?.detailed_status?.trim() || null;
  const episodePhase = ep.metadata_vars?.episode_phase ?? null;

  if (
    ep.status === EpisodeStatus.COMPLETED ||
    ep.status === EpisodeStatus.FAILED ||
    ep.status === EpisodeStatus.CANCELLED ||
    detailedStatus === EpisodeStatus.COMPLETED ||
    detailedStatus === EpisodeStatus.FAILED ||
    detailedStatus === EpisodeStatus.CANCELLED
  ) {
    return 100;
  }

  if (episodePhase && phaseProgressByEpisodePhase[episodePhase] !== undefined) {
    return phaseProgressByEpisodePhase[episodePhase] ?? null;
  }

  if (detailedStatus === "PLANNED") {
    return 25;
  }

  if (detailedStatus === "WAITING_USER") {
    return 45;
  }

  return null;
}

export default function Sidebar() {
  const { episodes, selectedEpisode, selectEpisode, createNewBenchmark, loading, setFeedbackState } = useEpisodes();
  const location = useLocation();
  const navigate = useNavigate();
  const [filter, setFilter] = useState("");
  const getEpisodeTitle = useCallback((ep: { task?: string | null; metadata_vars?: { prompt?: string | null } | null; id: string }) => {
    const taskTitle = ep.task?.trim();
    if (taskTitle) return taskTitle;
    const promptTitle = ep.metadata_vars?.prompt?.trim();
    if (promptTitle) return promptTitle;
    return ep.id.substring(0, 8);
  }, []);

  const filteredEpisodes = episodes.filter(ep => 
    getEpisodeTitle(ep).toLowerCase().includes(filter.toLowerCase()) || 
    ep.id.toLowerCase().includes(filter.toLowerCase())
  );

  const handleEpisodeClick = useCallback(async (id: string) => {
    await selectEpisode(id);

    const ep = episodes.find(e => e.id === id);
    if (ep && ep.metadata_vars?.episode_type === 'benchmark') {
      if (location.pathname !== '/benchmark' && location.pathname !== '/settings') {
        navigate('/benchmark');
      }
    } else if (ep) {
      if (location.pathname !== '/' && location.pathname !== '/settings') {
        navigate('/');
      }
    }

    if (location.pathname === '/settings') {
      // If we are in settings, we should go back to the appropriate UI.
      // For now, we go back to the previous page or default to workspace.
      navigate(-1);
    }
  }, [selectEpisode, episodes, location.pathname, navigate]);

  return (
    <div className="flex h-full w-full flex-col bg-card text-card-foreground">
      {/* App Header */}
      <div className="flex h-14 items-center border-b px-4 shrink-0">
        <div className="flex items-center gap-2 font-semibold">
          <div className="flex h-7 w-7 items-center justify-center rounded bg-primary text-primary-foreground">
            <Layers className="h-4 w-4" />
          </div>
          <span className="tracking-tight">Agentic CAD</span>
        </div>
      </div>

      {/* Main Navigation */}
      <div className="flex-1 overflow-hidden flex flex-col">
        <nav className="space-y-1 px-2 py-4 shrink-0">
          {navigation.map((item) => (
            <NavLink
              key={item.name}
              to={item.href}
              className={({ isActive }) =>
                cn(
                  "flex items-center gap-3 rounded-md px-3 py-2 text-sm font-medium transition-colors",
                  isActive
                    ? "bg-primary text-primary-foreground"
                    : "text-muted-foreground hover:bg-muted hover:text-foreground"
                )
              }
            >
              <item.icon className="h-4 w-4" />
              {item.name}
            </NavLink>
          ))}
        </nav>

        {/* Context-Aware History Section */}
        <div className="flex-1 flex flex-col min-h-0 border-t">
            <div className="p-4 space-y-3">
                <div className="flex items-center justify-between">
                    <h3 className="text-[10px] font-black uppercase tracking-widest text-muted-foreground">
                        {(location.pathname === '/' || location.pathname.startsWith('/settings')) ? 'Engineer History' : 'Benchmark Runs'}
                    </h3>
                    {(location.pathname === '/benchmark' || location.pathname === '/' || location.pathname.startsWith('/benchmark')) ? (
                        <Button 
                            size="sm" 
                            className="h-6 px-2 text-[9px] font-bold gap-1"
                            data-testid="create-new-button"
                            onClick={() => createNewBenchmark(location.pathname === '/benchmark')}
                        >
                            <Plus className="h-3 w-3" />
                            CREATE NEW
                        </Button>
                    ) : (
                        <History className="h-3 w-3 text-muted-foreground" />
                    )}
                </div>
                <div className="relative">
                    <Search className="absolute left-2 top-2.5 h-3.5 w-3.5 text-muted-foreground" />
                    <Input 
                      className="h-8 pl-8 text-xs bg-muted/50" 
                      placeholder="Filter sessions..." 
                      value={filter}
                      onChange={(e) => setFilter(e.target.value)}
                    />
                </div>
            </div>
            <div className="px-4 pb-3">
                <RevisionLineageSummary
                  episodes={episodes}
                  selectedEpisode={selectedEpisode}
                  onSelectEpisode={selectEpisode}
                />
            </div>
            <ScrollArea className="flex-1">
                <div className="p-2 space-y-1">
                    {loading ? (
                         <div className="p-4 text-xs text-muted-foreground text-center italic">Loading...</div>
                    ) : filteredEpisodes.length === 0 ? (
                         <div className="p-4 text-xs text-muted-foreground text-center">No history found.</div>
                    ) : (
                        filteredEpisodes.map(ep => (
                            <button 
                                key={ep.id} 
                                data-testid="sidebar-episode-item"
                                onClick={() => handleEpisodeClick(ep.id)}
                                className={cn(
                                  "relative w-full text-left p-2.5 rounded-md transition-all group border border-transparent overflow-hidden",
                                  selectedEpisode?.id === ep.id 
                                    ? "bg-primary/10 border-primary/20" 
                                    : "hover:bg-muted/50"
                                )}
                            >
                                <div className="flex justify-between items-start mb-1 min-w-0">
                                    <span className={cn(
                                      "text-[11px] font-semibold truncate flex-1 pr-2",
                                      selectedEpisode?.id === ep.id ? "text-primary" : "text-foreground"
                                    )}>
                                      {getEpisodeTitle(ep)}
                                    </span>
                                    <div data-testid="episode-status" data-status={ep.status}>
                                        {ep.status === EpisodeStatus.RUNNING ? (
                                            <div className="h-1.5 w-1.5 rounded-full bg-primary animate-pulse mt-1" />
                                        ) : ep.status === EpisodeStatus.COMPLETED ? (
                                            <CheckCircle2 className="h-3 w-3 text-green-500" />
                                        ) : (
                                            <XCircle className="h-3 w-3 text-destructive" />
                                        )}
                                    </div>
                                </div>
                                <div className="flex items-center gap-2 text-[9px] text-muted-foreground">
                                    <Clock className="h-2.5 w-2.5" />
                                    <span>{new Date(ep.created_at).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}</span>
                                    <span>•</span>
                                    <span className="font-mono">{ep.id.substring(0,4)}</span>
                                </div>

                                {(() => {
                                  const detailedStatus = ep.metadata_vars?.detailed_status?.trim() || null;
                                  const episodePhase = ep.metadata_vars?.episode_phase ?? null;
                                  const terminalReason = ep.metadata_vars?.terminal_reason?.trim() || null;
                                  const failureClass = ep.metadata_vars?.failure_class?.trim() || null;
                                  const progress = getEpisodeRailProgress(ep);
                                  const hasMetadataRail = !!detailedStatus || !!episodePhase || !!terminalReason || !!failureClass;

                                  return (
                                    <>
                                      {hasMetadataRail && (
                                        <div className="mt-2 flex flex-wrap items-center gap-1.5">
                                          {detailedStatus && (
                                            <span
                                              data-testid="sidebar-episode-detailed-status"
                                              data-detailed-status={detailedStatus}
                                              className="rounded-full border border-border/60 bg-background/80 px-2 py-0.5 text-[9px] font-black uppercase tracking-widest text-muted-foreground"
                                            >
                                              {detailedStatus}
                                            </span>
                                          )}
                                          {episodePhase && (
                                            <span
                                              data-testid="sidebar-episode-phase"
                                              data-episode-phase={episodePhase}
                                              className="rounded-full border border-border/60 bg-background/80 px-2 py-0.5 text-[9px] font-black uppercase tracking-widest text-muted-foreground"
                                            >
                                              {episodePhase}
                                            </span>
                                          )}
                                          {terminalReason && (
                                            <span
                                              data-testid="sidebar-episode-terminal-reason"
                                              data-terminal-reason={terminalReason}
                                              className="rounded-full border border-border/60 bg-background/80 px-2 py-0.5 text-[9px] font-black uppercase tracking-widest text-muted-foreground"
                                            >
                                              {terminalReason}
                                            </span>
                                          )}
                                          {failureClass && (
                                            <span
                                              data-testid="sidebar-episode-failure-class"
                                              data-failure-class={failureClass}
                                              className="rounded-full border border-border/60 bg-background/80 px-2 py-0.5 text-[9px] font-black uppercase tracking-widest text-muted-foreground"
                                            >
                                              {failureClass}
                                            </span>
                                          )}
                                        </div>
                                      )}

                                      {typeof progress === "number" && (
                                        <div className="mt-2 space-y-1">
                                          <div
                                            className="h-1.5 overflow-hidden rounded-full bg-muted"
                                            aria-label="Episode progress"
                                          >
                                            <div
                                              data-testid="sidebar-episode-progress"
                                              data-progress={progress}
                                              className={cn(
                                                "h-full rounded-full transition-all",
                                                ep.status === EpisodeStatus.FAILED
                                                  ? "bg-red-500"
                                                  : ep.status === EpisodeStatus.COMPLETED
                                                    ? "bg-emerald-500"
                                                    : "bg-primary",
                                              )}
                                              style={{ width: `${progress}%` }}
                                            />
                                          </div>
                                          <div className="flex items-center justify-between text-[9px] font-black uppercase tracking-widest text-muted-foreground">
                                            <span>Progress</span>
                                            <span>{progress}%</span>
                                          </div>
                                        </div>
                                      )}
                                    </>
                                  );
                                })()}
                                
                                {/* Hover Feedback Actions */}
                                <div className="absolute right-2 bottom-2 hidden group-hover:flex items-center gap-1.5 animate-in fade-in slide-in-from-right-1">
                                    <span
                                        data-testid="sidebar-thumbs-up"
                                        role="button"
                                        tabIndex={0}
                                        onClick={(e) => {
                                            e.stopPropagation();
                                            const isSelected = selectedEpisode?.id === ep.id;
                                            let lastTraceId = ep.last_trace_id;
                                            if (!lastTraceId && isSelected && selectedEpisode?.traces?.length) {
                                                lastTraceId = selectedEpisode.traces[selectedEpisode.traces.length - 1].id;
                                            } else if (!lastTraceId && ep.traces?.length) {
                                                lastTraceId = ep.traces[ep.traces.length - 1].id;
                                            }
                                            setFeedbackState({ traceId: lastTraceId || 0, score: 1 });
                                        }}
                                        onKeyDown={(e) => {
                                            if (e.key !== "Enter" && e.key !== " ") return;
                                            e.preventDefault();
                                            e.stopPropagation();
                                            const isSelected = selectedEpisode?.id === ep.id;
                                            let lastTraceId = ep.last_trace_id;
                                            if (!lastTraceId && isSelected && selectedEpisode?.traces?.length) {
                                                lastTraceId = selectedEpisode.traces[selectedEpisode.traces.length - 1].id;
                                            } else if (!lastTraceId && ep.traces?.length) {
                                                lastTraceId = ep.traces[ep.traces.length - 1].id;
                                            }
                                            setFeedbackState({ traceId: lastTraceId || 0, score: 1 });
                                        }}
                                        className="p-1 rounded-md bg-background/80 backdrop-blur shadow-sm border border-border/50 hover:text-green-500 transition-colors"
                                    >
                                        <ThumbsUp className="h-3 w-3" />
                                    </span>
                                    <span
                                        data-testid="sidebar-thumbs-down"
                                        role="button"
                                        tabIndex={0}
                                        onClick={(e) => {
                                            e.stopPropagation();
                                            const isSelected = selectedEpisode?.id === ep.id;
                                            let lastTraceId = ep.last_trace_id;
                                            if (!lastTraceId && isSelected && selectedEpisode?.traces?.length) {
                                                lastTraceId = selectedEpisode.traces[selectedEpisode.traces.length - 1].id;
                                            } else if (!lastTraceId && ep.traces?.length) {
                                                lastTraceId = ep.traces[ep.traces.length - 1].id;
                                            }
                                            setFeedbackState({ traceId: lastTraceId || 0, score: 0 });
                                        }}
                                        onKeyDown={(e) => {
                                            if (e.key !== "Enter" && e.key !== " ") return;
                                            e.preventDefault();
                                            e.stopPropagation();
                                            const isSelected = selectedEpisode?.id === ep.id;
                                            let lastTraceId = ep.last_trace_id;
                                            if (!lastTraceId && isSelected && selectedEpisode?.traces?.length) {
                                                lastTraceId = selectedEpisode.traces[selectedEpisode.traces.length - 1].id;
                                            } else if (!lastTraceId && ep.traces?.length) {
                                                lastTraceId = ep.traces[ep.traces.length - 1].id;
                                            }
                                            setFeedbackState({ traceId: lastTraceId || 0, score: 0 });
                                        }}
                                        className="p-1 rounded-md bg-background/80 backdrop-blur shadow-sm border border-border/50 hover:text-red-500 transition-colors"
                                    >
                                        <ThumbsDown className="h-3 w-3" />
                                    </span>
                                </div>
                            </button>
                        ))
                    )}
                </div>
            </ScrollArea>
        </div>
      </div>

      {/* Bottom Settings Navigation */}
      <div className="border-t p-4 shrink-0">
        <nav className="space-y-1">
          {/* 
          <NavLink
            to="/history"
            className={({ isActive }) =>
              cn(
                "flex items-center gap-3 rounded-md px-3 py-2 text-sm font-medium transition-colors",
                isActive
                  ? "bg-primary text-primary-foreground"
                  : "text-muted-foreground hover:bg-muted hover:text-foreground"
              )
            }
          >
            <History className="h-4 w-4" />
            History
          </NavLink>
          */}
          <NavLink
            to="/settings"
            className={({ isActive }) =>
              cn(
                "flex items-center gap-3 rounded-md px-3 py-2 text-sm font-medium transition-colors",
                isActive
                  ? "bg-primary text-primary-foreground"
                  : "text-muted-foreground hover:bg-muted hover:text-foreground"
              )
            }
          >
            <Settings className="h-4 w-4" />
            Settings
          </NavLink>
        </nav>
      </div>
    </div>
  );
}
