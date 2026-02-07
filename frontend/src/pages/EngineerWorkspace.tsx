import { useState } from 'react';
import { useEpisodes } from '../context/EpisodeContext';
import { useConnection } from '../context/ConnectionContext';
import { 
  XCircle, 
  Zap,
  Play,
  Box,
  BrainCircuit,
  Signal,
  SignalLow
} from "lucide-react";
import { Button } from "../components/ui/button";
import { Badge } from "../components/ui/badge";
import ReasoningTraces from '../components/workspace/ReasoningTraces';
import ArtifactView from '../components/workspace/ArtifactView';
import ConnectionError from '../components/shared/ConnectionError';

import ModelViewer from '../components/visualization/ModelViewer';

export default function EngineerWorkspace() {
  const { 
    selectedEpisode, 
    running, 
    startAgent, 
    setRunning 
  } = useEpisodes();
  const { isConnected } = useConnection();
  
  const [taskPrompt, setTaskPrompt] = useState('');
  const [resetTrigger, setResetTrigger] = useState(0);

  const handleRunAgent = async () => {
    // If no prompt is provided (since we removed the input), use a default task or current context
    await startAgent(taskPrompt || "Generate mechanical implementation for current specifications");
    setTaskPrompt('');
  };

  const handleInterrupt = () => {
    console.log("Interrupting execution...");
    setRunning(false);
  };

  return (
    <div className="flex flex-col h-full overflow-hidden bg-background">
        {/* Workspace Header */}
        <header className="flex shrink-0 items-center justify-between border-b px-6 h-16 bg-card/50 backdrop-blur-sm">
            <div className="flex items-center gap-4">
                <div className="size-10 flex items-center justify-center bg-primary/10 rounded-lg text-primary border border-primary/20">
                    <BrainCircuit className="h-6 w-6" />
                </div>
                <div>
                    <div className="flex items-center gap-2">
                        <h2 className="text-lg font-bold tracking-tight">Engineer Workspace</h2>
                        {isConnected ? (
                            <Badge variant="outline" className="text-[8px] h-4 px-1.5 border-green-500/30 text-green-500 bg-green-500/5 gap-1">
                                <Signal className="h-2 w-2" /> ONLINE
                            </Badge>
                        ) : (
                            <Badge variant="outline" className="text-[8px] h-4 px-1.5 border-red-500/30 text-red-500 bg-red-500/5 gap-1">
                                <SignalLow className="h-2 w-2" /> DISCONNECTED
                            </Badge>
                        )}
                    </div>
                    <div className="flex items-center gap-2">
                        <span className="text-[10px] font-black uppercase tracking-widest text-muted-foreground opacity-70">Agentic CAD Design</span>
                    </div>
                </div>
            </div>
            
            <div className="flex items-center gap-3">
                {running && (
                    <Button
                        onClick={handleInterrupt}
                        variant="destructive"
                        className="gap-2 h-10 px-4 font-bold"
                    >
                        <XCircle className="h-4 w-4" />
                        INTERRUPT
                    </Button>
                )}
                {!isConnected && (
                    <Badge variant="outline" className="text-[9px] h-6 px-3 font-bold border-red-500/30 text-red-500 bg-red-500/5 uppercase tracking-widest animate-pulse">
                        System Offline
                    </Badge>
                )}
                <Button
                    onClick={handleRunAgent}
                    disabled={running || !isConnected}
                    className="gap-2 h-10 px-6 font-bold"
                >
                    {running ? (
                      <Zap className="h-4 w-4 animate-pulse" />
                    ) : (
                      <Play className="h-4 w-4 fill-current" />
                    )}
                    {running ? 'RUNNING...' : 'SOLVE'}
                </Button>
            </div>
        </header>

        {/* Main Workspace Layout (Grid 9-cols for the Outlet area) */}
        <div className="flex-1 grid grid-cols-9 overflow-hidden">
            {/* Middle Column (span 3/9 of Outlet, which is 3/12 of Total) */}
            <div className="col-span-3 border-r overflow-hidden">
                <ReasoningTraces 
                  traces={selectedEpisode?.traces}
                  task={selectedEpisode?.task}
                  isRunning={running}
                  isConnected={isConnected}
                />
            </div>

            {/* Rightmost Column (span 6/9 of Outlet, which is 6/12 of Total) */}
            <main className="col-span-6 flex flex-col overflow-hidden">
                {/* Top: 3D view */}
                <div className="h-1/2 relative bg-gradient-to-b from-muted whitespace-nowrap overflow-hidden">
                   {/* 3D Metadata Overlays */}
                   <div className="absolute top-4 right-4 z-10 w-40 p-3 space-y-2 bg-background/80 backdrop-blur-md rounded-lg border shadow-sm">
                        <div className="flex justify-between items-center">
                            <span className="text-[10px] text-muted-foreground uppercase font-black">Est. Cost</span>
                            <span className="text-xs font-mono font-bold">${selectedEpisode?.metadata_vars?.cost ?? '0.00'}</span>
                        </div>
                        <div className="flex justify-between items-center border-t border-border/20 pt-2">
                            <span className="text-[10px] text-muted-foreground uppercase font-black">Weight</span>
                            <span className="text-xs font-mono font-bold">{selectedEpisode?.metadata_vars?.weight ?? '0'}g</span>
                        </div>
                    </div>

                    <div className="absolute bottom-4 left-4 z-10 flex gap-2">
                        <Button 
                          variant="secondary" 
                          size="icon" 
                          className="h-8 w-8 rounded-full shadow-lg border-primary/20"
                          onClick={() => setResetTrigger(prev => prev + 1)}
                        >
                          <Box className="h-4 w-4" />
                        </Button>
                    </div>

                    {/* Render Area */}
                    <div className="w-full h-full flex items-center justify-center overflow-hidden bg-muted/20 relative group">
                        {selectedEpisode?.assets && selectedEpisode.assets.filter(a => a.asset_type === 'video' || a.asset_type === 'image').length > 0 ? (
                            <div className="w-full h-full flex items-center justify-center p-6 relative">
                                {!isConnected && <ConnectionError className="absolute inset-0 z-50" />}
                                {selectedEpisode.assets.find(a => a.asset_type === 'video') ? (
                                    <video 
                                        src={selectedEpisode.assets.find(a => a.asset_type === 'video')?.s3_path} 
                                        controls 
                                        className="max-w-full max-h-full rounded-lg shadow-2xl ring-1 ring-white/10"
                                    />
                                ) : (
                                    <img 
                                        src={selectedEpisode.assets.find(a => a.asset_type === 'image')?.s3_path} 
                                        className="max-w-full max-h-full object-contain rounded-lg shadow-2xl ring-1 ring-white/10"
                                    />
                                )}
                            </div>
                        ) : (
                            <div className="w-full h-full relative">
                                <ModelViewer 
                                    className="w-full h-full" 
                                    assetUrl={selectedEpisode?.assets?.find(a => a.asset_type === 'stl' || a.asset_type === 'step')?.s3_path}
                                    isConnected={isConnected}
                                    resetTrigger={resetTrigger}
                                />
                                <div className="absolute top-4 left-1/2 -translate-x-1/2 z-10 pointer-events-none opacity-0 group-hover:opacity-100 transition-opacity">
                                    <Badge variant="outline" className="bg-background/50 backdrop-blur-sm text-[10px] uppercase font-bold tracking-widest px-3 py-1 border-primary/20 text-primary/70">
                                        Live Viewport • Isolated
                                    </Badge>
                                </div>
                            </div>
                        )}
                    </div>
                </div>

                {/* Bottom: Artifact view */}
                <div className="h-1/2 flex-1 border-t overflow-hidden">
                    <ArtifactView 
                      plan={selectedEpisode?.plan}
                      assets={selectedEpisode?.assets}
                      isConnected={isConnected}
                    />
                </div>
            </main>
        </div>
    </div>
  );
}
