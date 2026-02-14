import React, { createContext, useContext, useState, useEffect, useCallback } from "react";
import { fetchEpisodes, fetchEpisode, runAgent, generateBenchmark, updateBenchmarkObjectives, type Episode, type BenchmarkObjectives } from '../api/client';

interface EpisodeContextType {
  episodes: Episode[];
  selectedEpisode: Episode | null;
  loading: boolean;
  running: boolean;
  isCreationMode: boolean;
  activeArtifactId: string | null;
  setActiveArtifactId: (id: string | null) => void;
  refreshEpisodes: () => Promise<void>;
  selectEpisode: (id: string) => Promise<void>;
  startAgent: (task: string, objectives?: BenchmarkObjectives, metadata?: Record<string, unknown>) => Promise<void>;
  updateObjectives: (objectives: BenchmarkObjectives) => Promise<void>;
  interruptAgent: (id: string) => Promise<void>;
  setRunning: (running: boolean) => void;
  createNewBenchmark: (isBenchmark?: boolean) => void;
  clearSelection: () => void;
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
  }, []);

  const createNewBenchmark = useCallback((isBenchmark: boolean = false) => {
    setSelectedEpisode(null);
    setIsCreationMode(true);
    setIsBenchmarkCreation(isBenchmark);
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
          status: 'running' as any, // Cast to any to avoid enum issues for now, or import EpisodeStatus
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
  }, [isCreationMode, refreshEpisodes]);

  const interruptAgent = useCallback(async (id: string) => {
    try {
        await import('../api/client').then(m => m.interruptEpisode(id));
    } catch (e) {
        console.error("Failed to interrupt agent", e);
    }
  }, []);

  useEffect(() => {
    refreshEpisodes();
  }, [refreshEpisodes]);

  // Polling for active episodes
  useEffect(() => {
    let interval: number | undefined;
    if (selectedEpisode && (selectedEpisode.status === 'running' || running)) {
      interval = window.setInterval(async () => {
        try {
          const [episodesData, currentEp] = await Promise.all([
            fetchEpisodes(),
            fetchEpisode(selectedEpisode.id)
          ]);
          setEpisodes(episodesData);
          setSelectedEpisode(currentEp);
          if (currentEp.status !== 'running') {
            setRunning(false);
          }
        } catch (e) {
          console.error("Polling failed", e);
        }
      }, 3000);
    }
    return () => clearInterval(interval);
  }, [selectedEpisode, running]);

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
      interruptAgent,
      setRunning,
      createNewBenchmark,
      clearSelection,
      activeArtifactId,
      setActiveArtifactId,
      updateObjectives
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
