import React, { createContext, useContext, useState, useEffect } from 'react';
import { checkConnection } from '../api/client';

interface ConnectionContextType {
  isConnected: boolean;
  isMockMode: boolean;
}

const ConnectionContext = createContext<ConnectionContextType | undefined>(undefined);

export function ConnectionProvider({ children }: { children: React.ReactNode }) {
  const [isConnected, setIsConnected] = useState(true);
  const [isMockMode, setIsMockMode] = useState(false);

  useEffect(() => {
    // Initial check
    const initCheck = async () => {
        const { connected, isMockMode: mock } = await checkConnection();
        setIsConnected(connected);
        setIsMockMode(mock);
    };
    initCheck();

    // Connection polling
    const interval = setInterval(async () => {
        const { connected, isMockMode: mock } = await checkConnection();
        setIsConnected(connected);
        setIsMockMode(mock);
    }, 5000);

    return () => clearInterval(interval);
  }, []);

  return (
    <ConnectionContext.Provider value={{ isConnected, isMockMode }}>
      {children}
    </ConnectionContext.Provider>
  );
}

export function useConnection() {
  const context = useContext(ConnectionContext);
  if (context === undefined) {
    throw new Error('useConnection must be used within a ConnectionProvider');
  }
  return context;
}
