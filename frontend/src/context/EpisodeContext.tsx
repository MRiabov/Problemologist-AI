import React, { createContext, useContext, useState, useEffect, useCallback, useRef } from "react";
import { 
  fetchEpisodes, 
  fetchEpisode, 
  fetchEpisodeAsset,
  runAgent, 
  generateBenchmark, 
  updateBenchmarkObjectives, 
  continueEpisode, 
  confirmBenchmark as apiConfirmBenchmark,
  interruptEpisode as apiInterruptEpisode,
  submitEpisodeReview as apiSubmitEpisodeReview,
  type Episode, 
  type BenchmarkObjectives, 
  steerAgent as apiSteerAgent 
} from '../api/client';
import { EpisodeStatus } from '../api/generated/models/EpisodeStatus';
import { AssetType } from '../api/generated/models/AssetType';
import { OpenAPI } from '../api/generated/core/OpenAPI';
import type { TopologyNode } from "../components/visualization/ModelBrowser";

export interface ContextItem {
  id: string;
  type: 'code' | 'cad' | 'circuit';
  label: string;
  content?: string;
  metadata?: Record<string, any>;
}

interface EpisodeContextType {
  episodes: Episode[];
  selectedEpisode: Episode | null;
  loading: boolean;
  running: boolean;
  isCreationMode: boolean;
  activeArtifactId: string | null;
  selectedContext: ContextItem[];
  topologyNodes: TopologyNode[];
  feedbackState: { traceId: number; score: number } | null;
  setActiveArtifactId: (id: string | null) => void;
  setTopologyNodes: (nodes: TopologyNode[]) => void;
  setFeedbackState: (state: { traceId: number; score: number } | null) => void;
  refreshEpisodes: () => Promise<void>;
  selectEpisode: (id: string) => Promise<void>;
  startAgent: (task: string, objectives?: BenchmarkObjectives, metadata?: Record<string, unknown>) => Promise<void>;
  continueAgent: (id: string, message: string, metadata?: Record<string, unknown>) => Promise<void>;
  steerAgent: (id: string, text: string, metadata?: Record<string, any>) => Promise<void>;
  confirmBenchmark: (id: string, comment?: string) => Promise<void>;
  submitReview: (id: string, reviewContent: string) => Promise<void>;
  updateObjectives: (objectives: BenchmarkObjectives) => Promise<void>;
  interruptAgent: (id: string) => Promise<void>;
  setRunning: (running: boolean) => void;
  createNewBenchmark: (isBenchmark?: boolean) => void;
  clearSelection: () => void;
  addToContext: (item: ContextItem) => void;
  removeFromContext: (id: string) => void;
  clearContext: () => void;
}

const EpisodeContext = createContext<EpisodeContextType | undefined>(undefined);

export function EpisodeProvider({ children }: { children: React.ReactNode }) {
  const [episodes, setEpisodes] = useState<Episode[]>([]);
  const [selectedEpisode, setSelectedEpisode] = useState<Episode | null>(null);
  const [loading, setLoading] = useState(true);
  const [running, setRunning] = useState(false);
  const [isCreationMode, setIsCreationMode] = useState(false);
  const [, setIsBenchmarkCreation] = useState(false);
  const [activeArtifactId, setActiveArtifactId] = useState<string | null>(null);
  const [topologyNodes, setTopologyNodes] = useState<TopologyNode[]>([]);
  const [feedbackState, setFeedbackState] = useState<{ traceId: number; score: number } | null>(null);
  const isCreationModeRef = useRef(false);
  const isBenchmarkCreationRef = useRef(false);

  const hydrateEpisodeAssets = useCallback(async (episode: Episode): Promise<Episode> => {
    const assets = episode.assets ?? [];
    const hydrateTargets = assets.filter((asset) => {
      if (asset.content) {
        return false;
      }

      const path = asset.s3_path.toLowerCase();
      if (
        path.endsWith(".json") ||
        path.endsWith(".yaml") ||
        path.endsWith(".yml") ||
        path.endsWith(".md") ||
        path.endsWith(".txt")
      ) {
        return true;
      }

      return (
        asset.asset_type === AssetType.MARKDOWN ||
        asset.asset_type === AssetType.LOG ||
        asset.asset_type === AssetType.ERROR ||
        asset.asset_type === AssetType.PYTHON ||
        asset.asset_type === AssetType.MJCF ||
        asset.asset_type === AssetType.OTHER
      );
    });

    if (hydrateTargets.length === 0) {
      return episode;
    }

    const hydratedAssets = await Promise.all(
      hydrateTargets.map(async (asset) => {
        try {
          const content = await fetchEpisodeAsset(episode.id, asset.s3_path);
          return { ...asset, content };
        } catch (error) {
          console.error("Failed to hydrate episode asset content", {
            episodeId: episode.id,
            path: asset.s3_path,
            error,
          });
          return asset;
        }
      }),
    );

    const hydratedByPath = new Map(
      hydratedAssets.map((asset) => [asset.s3_path, asset]),
    );

    return {
      ...episode,
      assets: assets.map((asset) => hydratedByPath.get(asset.s3_path) ?? asset),
    };
  }, []);

  // WP10: Restore selected episode from localStorage on mount
  useEffect(() => {
    const savedId = localStorage.getItem("selectedEpisodeId");
    if (savedId && !selectedEpisode) {
        fetchEpisode(savedId).then(async data => {
            setSelectedEpisode(await hydrateEpisodeAssets(data));
        }).catch(err => {
            console.error("Failed to restore episode from localStorage:", err);
            localStorage.removeItem("selectedEpisodeId");
        });
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const refreshEpisodes = useCallback(async () => {
    try {
      const data = await fetchEpisodes();
      setEpisodes(data);
      
      // Always refresh selected episode from the detail endpoint because
      // listEpisodes intentionally omits traces/assets for payload size.
      if (selectedEpisode) {
          const detailed = await fetchEpisode(selectedEpisode.id);
          const hydratedDetailed = await hydrateEpisodeAssets(detailed);
          setSelectedEpisode(prev => {
            if (!prev) return hydratedDetailed;
            return {
              ...prev,
              ...hydratedDetailed,
              traces: hydratedDetailed.traces || [],
              assets: hydratedDetailed.assets || [],
            };
          });
      }
    } catch (e) {
      console.error("Failed to fetch episodes", e);
    } finally {
      setLoading(false);
    }
  }, [
    selectedEpisode?.id,
    selectedEpisode?.status,
    selectedEpisode?.metadata_vars?.detailed_status,
    selectedEpisode?.traces?.length,
    selectedEpisode?.assets?.length
  ]);

  const selectEpisode = useCallback(async (id: string) => {
    setIsCreationMode(false);
    setIsBenchmarkCreation(false);
    isCreationModeRef.current = false;
    isBenchmarkCreationRef.current = false;
    setRunning(false);
    try {
      const fullEp = await fetchEpisode(id);
      setSelectedEpisode(await hydrateEpisodeAssets(fullEp));
      localStorage.setItem("selectedEpisodeId", id);
    } catch (e) {
      console.error("Failed to fetch episode details", e);
      const ep = episodes.find(e => e.id === id);
      if (ep) {
          setSelectedEpisode(await hydrateEpisodeAssets(ep));
          localStorage.setItem("selectedEpisodeId", id);
      }
    }
  }, [episodes]);

  const clearSelection = useCallback(() => {
    setSelectedEpisode(null);
    localStorage.removeItem("selectedEpisodeId");
    setIsCreationMode(false);
    setIsBenchmarkCreation(false);
    isCreationModeRef.current = false;
    isBenchmarkCreationRef.current = false;
    setTopologyNodes([]);
    setRunning(false);
  }, []);

  const createNewBenchmark = useCallback((isBenchmark: boolean = false) => {
    setSelectedEpisode(null);
    localStorage.removeItem("selectedEpisodeId");
    setIsCreationMode(true);
    setIsBenchmarkCreation(isBenchmark);
    isCreationModeRef.current = true;
    isBenchmarkCreationRef.current = isBenchmark;
    setTopologyNodes([]);
    setRunning(false);
  }, []);

  const updateObjectives = useCallback(async (objectives: BenchmarkObjectives) => {
    if (!selectedEpisode) return;
    try {
      await updateBenchmarkObjectives(selectedEpisode.id, objectives);
      // Optionally refresh episode to show updated state if backend returns it
    } catch (e) {
      console.error("Failed to update objectives", e);
      throw e;
    }
  }, [selectedEpisode]);

  const startAgent = useCallback(async (task: string, objectives?: BenchmarkObjectives, metadata?: Record<string, unknown>) => {
    setRunning(true);
    const wasBenchmarkCreation = isBenchmarkCreationRef.current;
    setIsCreationMode(false);
    setIsBenchmarkCreation(false);
    isCreationModeRef.current = false;
    isBenchmarkCreationRef.current = false;
    
    try {
      let response;
      const isSolvingBenchmark = metadata && metadata.benchmark_id;

      if (wasBenchmarkCreation && !isSolvingBenchmark) {
         response = await generateBenchmark(task, objectives);
         // Backend returns { session_id: ... }
         if (response.session_id) {
            response.episode_id = response.session_id; // Normalize key
         }
      } else {
         const sessionId = `sess-${Math.random().toString(36).substring(2, 10)}`;
         response = await runAgent(task, sessionId, metadata);
      }
      
      if (response.episode_id) {
        console.log("Agent started, episode_id:", response.episode_id);
        // Create a minimal episode object to start polling
        const newEpisode: Episode = {
          id: response.episode_id,
          task: task,
          status: EpisodeStatus.RUNNING,
          created_at: new Date().toISOString(),
          updated_at: new Date().toISOString(),
          traces: [],
          assets: [],
          plan: '',
          metadata_vars: {}
        };
        setSelectedEpisode(newEpisode);
        localStorage.setItem("selectedEpisodeId", response.episode_id);
        await refreshEpisodes();
      } else {
        console.error("No episode_id in response:", response);
      }
    } catch (e) {
      console.error("Failed to run agent", e);
      setRunning(false);
    }
  }, [refreshEpisodes]);

  const continueAgent = useCallback(async (id: string, message: string, metadata?: Record<string, unknown>) => {
    setRunning(true);
    try {
      await continueEpisode(id, message, metadata);
      // Wait a bit then refresh to show new trace if polling didn't catch it
      await refreshEpisodes();
    } catch (e) {
      console.error("Failed to continue agent", e);
      setRunning(false);
    }
  }, [refreshEpisodes]);

  const steerAgent = useCallback(async (id: string, text: string, metadata?: Record<string, any>) => {
    try {
      await apiSteerAgent(id, text, metadata);
    } catch (e) {
      console.error("Failed to steer agent", e);
    }
  }, []);

  const confirmBenchmark = useCallback(async (id: string, comment?: string) => {
    setRunning(true);
    try {
        await apiConfirmBenchmark(id, comment);
        await refreshEpisodes();
    } catch (e) {
        console.error("Failed to confirm benchmark", e);
        setRunning(false);
    }
  }, [refreshEpisodes]);

  const submitReview = useCallback(async (id: string, reviewContent: string) => {
    try {
      await apiSubmitEpisodeReview(id, reviewContent);
      await refreshEpisodes();
    } catch (e) {
      console.error("Failed to submit episode review", e);
      throw e;
    }
  }, [refreshEpisodes]);

  const [selectedContext, setSelectedContext] = useState<ContextItem[]>([]);

  const addToContext = useCallback((item: ContextItem) => {
    setSelectedContext((prev) => {
      if (prev.find(i => i.id === item.id)) return prev;
      return [...prev, item];
    });
  }, []);

  const removeFromContext = useCallback((id: string) => {
    setSelectedContext((prev) => prev.filter(i => i.id !== id));
  }, []);

  const clearContext = useCallback(() => {
    setSelectedContext([]);
  }, []);

  const interruptAgent = useCallback(async (id: string) => {
    try {
        await apiInterruptEpisode(id);
    } catch (e) {
        console.error("Failed to interrupt agent", e);
    }
  }, []);

  useEffect(() => {
    refreshEpisodes();
    // Regular polling for episode list updates (e.g. new sessions created elsewhere)
    const listInterval = setInterval(refreshEpisodes, import.meta.env.VITE_IS_INTEGRATION_TEST === 'true' ? 2000 : 10000);
    return () => clearInterval(listInterval);
  }, [refreshEpisodes]);

  const selectedEpisodeIdRef = useRef<string | null>(null);
  useEffect(() => {
    selectedEpisodeIdRef.current = selectedEpisode?.id || null;
  }, [selectedEpisode?.id]);

  // WebSocket subscription for real-time updates
  useEffect(() => {
    if (!selectedEpisode) return;
    
    // T026: Store the ID this connection belongs to to avoid leakage from other sessions
    const connectionEpisodeId = selectedEpisode.id;

    let wsUrl: string;
    const base = OpenAPI.BASE || '';
    
    if (base.startsWith('http')) {
        // Absolute URL (standard in integration tests / production)
        wsUrl = base.replace(/^http/, 'ws') + `/api/episodes/${connectionEpisodeId}/ws`;
    } else {
        // Relative URL or empty (standard in dev proxy mode)
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const host = window.location.host;
        const prefix = base || '/api';
        wsUrl = `${protocol}//${host}${prefix}/episodes/${connectionEpisodeId}/ws`;
    }
    
    let ws: WebSocket;
    let reconnectTimeout: number;

    const connect = () => {
      ws = new WebSocket(wsUrl);

      ws.onmessage = (event: MessageEvent) => {
        try {
          const message = JSON.parse(event.data);
          if (message.type === 'new_trace') {
            setSelectedEpisode((prev: Episode | null) => {
              // Map payload to TraceResponse structure
              const newTrace = {
                id: message.id,
                langfuse_trace_id: message.langfuse_trace_id,
                trace_type: message.trace_type,
                name: message.name,
                content: message.content,
                metadata_vars: message.metadata,
                created_at: message.created_at
              };

              // T026: Only update if the message is for the currently selected episode
              // AND this connection matches the message's origin (implicit in connectionEpisodeId)
              if (!prev || prev.id !== connectionEpisodeId || prev.id !== selectedEpisodeIdRef.current) return prev;

              const currentTraces = prev.traces || [];

              // Avoid duplicates
              if (currentTraces.some((t: any) => t.id === newTrace.id)) return prev;
              
              return {
                ...prev,
                last_trace_id: newTrace.id,
                traces: [...currentTraces, newTrace]
              };
            });
          } else if (message.type === 'status_update') {
            setSelectedEpisode((prev: Episode | null) => {
              if (!prev || prev.id !== connectionEpisodeId || prev.id !== selectedEpisodeIdRef.current) return prev;
              return { 
                ...prev, 
                status: message.status,
                metadata_vars: message.metadata_vars || prev.metadata_vars
              };
            });
            // Only set running state if this update is for the ACTIVE episode
            if (connectionEpisodeId === selectedEpisodeIdRef.current) {
                if (message.status !== EpisodeStatus.RUNNING) {
                  setRunning(false);
                  // Hydrate full episode payload to avoid missing traces/assets after completion.
                  fetchEpisode(connectionEpisodeId)
                    .then(async (fullEp) => {
                      const hydratedFullEp = await hydrateEpisodeAssets(fullEp);
                      setSelectedEpisode((prev: Episode | null) => {
                        if (!prev || prev.id !== connectionEpisodeId || prev.id !== selectedEpisodeIdRef.current) {
                          return prev;
                        }
                        return hydratedFullEp;
                      });
                      setEpisodes(prevEpisodes => prevEpisodes.map(ep => ep.id === connectionEpisodeId ? { ...ep, ...hydratedFullEp } : ep));
                    })
                    .catch((e) => {
                      console.error("Failed to hydrate episode after status update", e);
                    });
                }
            }
          } else if (message.type === 'file_update') {
            setSelectedEpisode((prev: Episode | null) => {
                if (!prev || prev.id !== connectionEpisodeId || prev.id !== selectedEpisodeIdRef.current) return prev;
                
                // Real-time plan updates
                const path = message.data.path || '';
                if (path.toLowerCase().endsWith('plan.md')) {
                    return { ...prev, plan: message.data.content };
                }

                // Update content of existing assets or add new ones
                const currentAssets = prev.assets || [];
                let assetFound = false;
                const updatedAssets = currentAssets.map((a: any) => {
                    if (a.s3_path === message.data.path || ('/' + a.s3_path) === message.data.path) {
                        assetFound = true;
                        return { ...a, content: message.data.content };
                    }
                    return a;
                });

                if (!assetFound) {
                    updatedAssets.push({
                        id: message.data.id || `temp-${Date.now()}-${Math.random()}`,
                        s3_path: message.data.path,
                        content: message.data.content,
                        asset_type: message.data.asset_type,
                        created_at: message.data.created_at || new Date().toISOString()
                    });
                }
                
                return { ...prev, assets: updatedAssets };
            });
          }
          
          // Also update the episode in the sidebar list if it's the connected one
          setEpisodes(prev => prev.map(ep => {
            if (ep.id === connectionEpisodeId) {
              const newTraceId = message.type === 'new_trace' ? message.id : ep.last_trace_id;
              return {
                ...ep,
                status: message.type === 'status_update' ? message.status : ep.status,
                metadata_vars: message.type === 'status_update'
                  ? (message.metadata_vars || ep.metadata_vars)
                  : ep.metadata_vars,
                last_trace_id: newTraceId
              };
            }
            return ep;
          }));
        } catch (e) {
          console.error("WS message parse failed", e);
        }
      };

      ws.onclose = () => {
        // Simple exponential backoff for reconnection
        reconnectTimeout = window.setTimeout(connect, 3000);
      };

      ws.onerror = (err) => {
        console.error("WebSocket error", err);
        ws.close();
      };
    };

    connect();

    return () => {
      if (ws) {
        ws.onclose = null; // Prevent reconnect on intentional close
        ws.close();
      }
      clearTimeout(reconnectTimeout);
    };
  }, [selectedEpisode?.id]);

  // Fallback polling for active episode details (slower than before)
  useEffect(() => {
    let interval: number | undefined;
    if (selectedEpisode && (running || selectedEpisode.status === EpisodeStatus.RUNNING)) {
      interval = window.setInterval(async () => {
        try {
          const [episodesData, currentEp] = await Promise.all([
            fetchEpisodes(),
            fetchEpisode(selectedEpisode.id)
          ]);
          setEpisodes(episodesData);
          const hydratedCurrentEp = await hydrateEpisodeAssets(currentEp);

          // Only update if something changed (avoid re-rendering everything if WS handled it)
          setSelectedEpisode((prev: Episode | null) => {
             if (!prev) return hydratedCurrentEp;
             // Check for meaningful changes to avoid re-rendering components like buttons
             const prevDetailedStatus = prev.metadata_vars?.detailed_status || null;
             const currentDetailedStatus = hydratedCurrentEp.metadata_vars?.detailed_status || null;
             if (prev.id === currentEp.id &&
                 prev.status === hydratedCurrentEp.status &&
                 prevDetailedStatus === currentDetailedStatus &&
                 prev.last_trace_id === hydratedCurrentEp.last_trace_id &&
                 (prev.assets?.length || 0) === (hydratedCurrentEp.assets?.length || 0) &&
                 (prev.plan === hydratedCurrentEp.plan)) {
                 return prev;
             }
             console.log("Updating selectedEpisode from poller", { old: prev.status, new: hydratedCurrentEp.status });
             return hydratedCurrentEp;
          });

          if (hydratedCurrentEp.status !== EpisodeStatus.RUNNING) {
            setRunning(false);
          }
        } catch (e) {
          console.error("Polling failed", e);
        }
      }, import.meta.env.VITE_IS_INTEGRATION_TEST === 'true' ? 3000 : 10000);
    }
    return () => clearInterval(interval);
  }, [selectedEpisode?.id, running]);

  return (
    <EpisodeContext.Provider value={{
      episodes,
      selectedEpisode,
      loading,
      running,
      isCreationMode,
      refreshEpisodes,
      selectEpisode,
      startAgent,
      continueAgent,
      steerAgent,
      confirmBenchmark,
      submitReview,
      interruptAgent,
      setRunning,
      createNewBenchmark,
      clearSelection,
      activeArtifactId,
      setActiveArtifactId,
      updateObjectives,
      selectedContext,
      addToContext,
      removeFromContext,
      clearContext,
      topologyNodes,
      setTopologyNodes,
      feedbackState,
      setFeedbackState
    }}>
      {children}
    </EpisodeContext.Provider>
  );
}

export function useEpisodes() {
  const context = useContext(EpisodeContext);
  if (context === undefined) {
    throw new Error('useEpisodes must be used within an EpisodeProvider');
  }
  return context;
}
