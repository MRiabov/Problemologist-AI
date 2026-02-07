import React, { createContext, useContext, useState, useEffect, useCallback } from 'react';
import { fetchEpisodes, fetchEpisode, runAgent, type Episode } from '../api/client';

interface EpisodeContextType {
  episodes: Episode[];
  selectedEpisode: Episode | null;
  loading: boolean;
  running: boolean;
  refreshEpisodes: () => Promise<void>;
  selectEpisode: (id: string) => Promise<void>;
  startAgent: (task: string) => Promise<void>;
  setRunning: (running: boolean) => void;
}

const EpisodeContext = createContext<EpisodeContextType | undefined>(undefined);

export function EpisodeProvider({ children }: { children: React.ReactNode }) {
  const [episodes, setEpisodes] = useState<Episode[]>([]);
  const [selectedEpisode, setSelectedEpisode] = useState<Episode | null>(null);
  const [loading, setLoading] = useState(true);
  const [running, setRunning] = useState(false);

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
    try {
      const fullEp = await fetchEpisode(id);
      setSelectedEpisode(fullEp);
    } catch (e) {
      console.error("Failed to fetch episode details", e);
      const ep = episodes.find(e => e.id === id);
      if (ep) setSelectedEpisode(ep);
    }
  }, [episodes]);

  const startAgent = useCallback(async (task: string) => {
    setRunning(true);
    try {
      const sessionId = `sess-${Math.random().toString(36).substring(2, 10)}`;
      const response = await runAgent(task, sessionId);
      
      if (response.episode_id) {
        // Create a minimal episode object to start polling
        const newEpisode: Episode = {
          id: response.episode_id,
          task: task,
          status: 'running',
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
  }, [refreshEpisodes]);

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
      refreshEpisodes,
      selectEpisode,
      startAgent,
      setRunning
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
