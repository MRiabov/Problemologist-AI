import React, { createContext, useContext, ReactNode } from 'react';
import { useRun } from '@/hooks/api/useRuns';
import { Episode } from '@/api';

interface RunContextType {
  runId: string;
  run: Episode | undefined;
  isLoading: boolean;
  error: Error | null;
}

const RunContext = createContext<RunContextType | undefined>(undefined);

interface RunProviderProps {
  runId: string;
  children: ReactNode;
}

export function RunProvider({ runId, children }: RunProviderProps) {
  const { data: run, isLoading, error } = useRun(runId);

  const value = {
    runId,
    run,
    isLoading,
    error: error as Error | null,
  };

  return <RunContext.Provider value={value}>{children}</RunContext.Provider>;
}

export function useRunContext() {
  const context = useContext(RunContext);
  if (context === undefined) {
    throw new Error('useRunContext must be used within a RunProvider');
  }
  return context;
}
