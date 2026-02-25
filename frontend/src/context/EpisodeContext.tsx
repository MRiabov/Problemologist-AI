import React, { createContext, useContext, useState, useEffect, useCallback } from "react";
import { fetchEpisodes, fetchEpisode, runAgent, generateBenchmark, updateBenchmarkObjectives, continueEpisode, type Episode, type BenchmarkObjectives, steerAgent as apiSteerAgent } from '../api/client';
import { EpisodeStatus } from '../api/generated/models/EpisodeStatus';
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
  const [isBenchmarkCreation, setIsBenchmarkCreation] = useState(false);
  const [activeArtifactId, setActiveArtifactId] = useState<string | null>(null);
  const [topologyNodes, setTopologyNodes] = useState<TopologyNode[]>([]);
  const [feedbackState, setFeedbackState] = useState<{ traceId: number; score: number } | null>(null);

  const refreshEpisodes = useCallback(async () => {
    try {
      const data = await fetchEpisodes();
      setEpisodes(data);
    } catch (e) {
      console.error("Failed to fetch episodes", e);
    } finally {
      setLoading(false);
    }
  }, []);

  const selectEpisode = useCallback(async (id: string) => {
    setIsCreationMode(false);
    setIsBenchmarkCreation(false);
    setRunning(false);
    try {
      const fullEp = await fetchEpisode(id);
      setSelectedEpisode(fullEp);
    } catch (e) {
      console.error("Failed to fetch episode details", e);
      const ep = episodes.find(e => e.id === id);
      if (ep) setSelectedEpisode(ep);
    }
  }, [episodes]);

  const clearSelection = useCallback(() => {
    setSelectedEpisode(null);
    setIsCreationMode(false);
    setIsBenchmarkCreation(false);
    setTopologyNodes([]);
    setRunning(false);
  }, []);

  const createNewBenchmark = useCallback((isBenchmark: boolean = false) => {
    setSelectedEpisode(null);
    setIsCreationMode(true);
    setIsBenchmarkCreation(isBenchmark);
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
    const wasCreationMode = isCreationMode;
    const wasBenchmarkCreation = isBenchmarkCreation;
    setIsCreationMode(false);
    setIsBenchmarkCreation(false);
    
    try {
      let response;
      const isSolvingBenchmark = metadata && metadata.benchmark_id;

      if ((wasBenchmarkCreation || (wasCreationMode && objectives)) && !isSolvingBenchmark) {
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
        // Create a minimal episode object to start polling
        const newEpisode: Episode = {
          id: response.episode_id,
          task: task,
          status: EpisodeStatus.RUNNING,
          created_at: new Date().toISOString(),
          updated_at: new Date().toISOString(),
          traces: [],
          assets: []
        };
        setSelectedEpisode(newEpisode);
        await refreshEpisodes();
      }
    } catch (e) {
      console.error("Failed to run agent", e);
      setRunning(false);
    }
  }, [isCreationMode, refreshEpisodes, isBenchmarkCreation]);

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
        await import('../api/client').then(m => m.confirmBenchmark(id, comment));
        await refreshEpisodes();
    } catch (e) {
        console.error("Failed to confirm benchmark", e);
        setRunning(false);
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
        await import('../api/client').then(m => m.interruptEpisode(id));
    } catch (e) {
        console.error("Failed to interrupt agent", e);
    }
  }, []);

  useEffect(() => {
    refreshEpisodes();
    // Regular polling for episode list updates (e.g. new sessions created elsewhere)
    const listInterval = setInterval(refreshEpisodes, 10000);
    return () => clearInterval(listInterval);
  }, [refreshEpisodes]);

  // WebSocket subscription for real-time updates
  useEffect(() => {
    if (!selectedEpisode) return;

    let wsUrl: string;
    const base = OpenAPI.BASE || '';
    
    if (base.startsWith('http')) {
        // Absolute URL (standard in integration tests / production)
        wsUrl = base.replace(/^http/, 'ws') + `/episodes/${selectedEpisode.id}/ws`;
    } else {
        // Relative URL or empty (standard in dev proxy mode)
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const host = window.location.host;
        const prefix = base || '/api';
        wsUrl = `${protocol}//${host}${prefix}/episodes/${selectedEpisode.id}/ws`;
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
              if (!prev || prev.id !== selectedEpisode.id) return prev;
              
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

              const currentTraces = prev.traces || [];

              // Avoid duplicates
              if (currentTraces.some((t: any) => t.id === newTrace.id)) return prev;
              
              return {
                ...prev,
                traces: [...currentTraces, newTrace]
              };
            });
          } else if (message.type === 'status_update') {
            setSelectedEpisode((prev: Episode | null) => {
              if (!prev || prev.id !== selectedEpisode.id) return prev;
              return { 
                ...prev, 
                status: message.status,
                metadata_vars: message.metadata_vars || prev.metadata_vars
              };
            });
            if (message.status !== EpisodeStatus.RUNNING) {
              setRunning(false);
            }
          } else if (message.type === 'file_update') {
            setSelectedEpisode((prev: Episode | null) => {
                if (!prev || prev.id !== selectedEpisode.id) return prev;
                
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

                if (!assetFound && message.data.id) {
                    updatedAssets.push({
                        id: message.data.id,
                        s3_path: message.data.path,
                        content: message.data.content,
                        asset_type: message.data.asset_type,
                        created_at: message.data.created_at || new Date().toISOString()
                    });
                }
                
                return { ...prev, assets: updatedAssets };
            });
          }
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
    if (selectedEpisode && (selectedEpisode.status === EpisodeStatus.RUNNING || running)) {
      interval = window.setInterval(async () => {
        try {
          const [episodesData, currentEp] = await Promise.all([
            fetchEpisodes(),
            fetchEpisode(selectedEpisode.id)
          ]);
          setEpisodes(episodesData);
          
          // Only update if something changed (avoid re-rendering everything if WS handled it)
          setSelectedEpisode((prev: Episode | null) => {
             if (!prev) return currentEp;
             if (JSON.stringify(prev) === JSON.stringify(currentEp)) return prev;
             return currentEp;
          });

          if (currentEp.status !== EpisodeStatus.RUNNING) {
            setRunning(false);
          }
        } catch (e) {
          console.error("Polling failed", e);
        }
      }, 10000);
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
