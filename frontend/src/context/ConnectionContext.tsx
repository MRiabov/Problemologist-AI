import React, { createContext, useContext, useState, useEffect } from 'react';
import { checkConnection } from '../api/client';

interface ConnectionContextType {
  isConnected: boolean;
}

const ConnectionContext = createContext<ConnectionContextType | undefined>(undefined);

export function ConnectionProvider({ children }: { children: React.ReactNode }) {
  const [isConnected, setIsConnected] = useState(true);

  useEffect(() => {
    // Initial check
    const initCheck = async () => {
        const connected = await checkConnection();
        setIsConnected(connected);
    };
    initCheck();

    // Connection polling
    const interval = setInterval(async () => {
        const connected = await checkConnection();
        setIsConnected(connected);
    }, 5000);

    return () => clearInterval(interval);
  }, []);

  return (
    <ConnectionContext.Provider value={{ isConnected }}>
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
