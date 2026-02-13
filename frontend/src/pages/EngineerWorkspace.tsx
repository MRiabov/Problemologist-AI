import { useState } from 'react';
import { useEpisodes } from '../context/EpisodeContext';
import { useConnection } from '../context/ConnectionContext';
import { 
  XCircle, 
  BrainCircuit,
  Box,
  SignalLow,
  AlertCircle
} from "lucide-react";
import { Button } from "../components/ui/button";
import { Badge } from "../components/ui/badge";
import ChatWindow from '../components/workspace/ChatWindow';
import ArtifactView from '../components/workspace/ArtifactView';
import ConnectionError from '../components/shared/ConnectionError';
import { 
  ResizableHandle, 
  ResizablePanel, 
  ResizablePanelGroup 
} from "../components/ui/resizable";

import ModelViewer from '../components/visualization/ModelViewer';
export default function EngineerWorkspace() {
  const { 
    selectedEpisode, 
    running, 
    setRunning 
  } = useEpisodes();
  const { isConnected } = useConnection();
  const [resetTrigger, setResetTrigger] = useState(0);

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
                        {!isConnected && (
                            <Badge variant="outline" className="text-[8px] h-4 px-1.5 border-red-500/30 text-red-500 bg-red-500/5 gap-1">
                                <SignalLow className="h-2 w-2" /> OFFLINE
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
            </div>
        </header>

        {/* Error Alert */}
        {selectedEpisode?.status === 'failed' && (
          <div className="bg-red-500/10 border-b border-red-500/20 px-6 py-3 flex items-center gap-3 animate-in fade-in slide-in-from-top-1">
            <AlertCircle className="h-5 w-5 text-red-500 shrink-0" />
            <div className="flex-1">
              <p className="text-sm font-bold text-red-500 uppercase tracking-tight">Agent Execution Failed</p>
              <p className="text-xs text-red-400/80 font-medium">
                The agent encountered an error. Check the traces for more details.
              </p>
            </div>
            <Button 
              variant="ghost" 
              size="sm" 
              className="h-8 text-[10px] font-bold uppercase text-red-400 hover:text-red-300 hover:bg-red-500/10"
              onClick={() => {
                // We don't have a local error state here, but we can't easily "dismiss" a status from the backend
                // without updating the backend. So we just leave it for now or provide a close button that hides it locally.
              }}
            >
              Details Below
            </Button>
          </div>
        )}

        {/* Main Workspace Layout - Resizable Columns */}
        <div className="flex-1 overflow-hidden">
            <ResizablePanelGroup 
              orientation="horizontal" 
              className="h-full w-full"
              onLayoutChanged={(layout) => {
                localStorage.setItem('resizable-layout:workspace-cols', JSON.stringify(layout));
              }}
            >
                {/* Chat Column */}
                <ResizablePanel 
                  defaultSize={(() => {
                    const saved = localStorage.getItem('resizable-layout:workspace-cols');
                    if (saved) {
                      try {
                        const layout = JSON.parse(saved);
                        return layout[0] ?? 33;
                      } catch (e) { return 33; }
                    }
                    return 33;
                  })()} 
                  minSize={0} 
                  maxSize={95}
                  collapsible={true}
                  className="overflow-hidden"
                >
                    <ChatWindow 
                      traces={selectedEpisode?.traces}
                      task={selectedEpisode?.task}
                      isRunning={running}
                      isConnected={isConnected}
                    />
                </ResizablePanel>

                <ResizableHandle withHandle />

                {/* Rightmost Column (main area) with Vertical Split */}
                <ResizablePanel defaultSize={67} className="min-w-0">
                    <ResizablePanelGroup 
                      orientation="vertical" 
                      className="h-full w-full"
                      onLayoutChanged={(layout) => {
                        localStorage.setItem('resizable-layout:workspace-rows', JSON.stringify(layout));
                      }}
                    >
                        {/* Top: 3D view */}
                        <ResizablePanel 
                          defaultSize={(() => {
                            const saved = localStorage.getItem('resizable-layout:workspace-rows');
                            if (saved) {
                              try {
                                const layout = JSON.parse(saved);
                                return layout[0] ?? 50;
                              } catch (e) { return 50; }
                            }
                            return 50;
                          })()} 
                          minSize={30}
                        >
                            <div className="h-full relative bg-gradient-to-b from-muted whitespace-nowrap overflow-hidden">
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
                                                    className="max-w-full max-h-full rounded-lg shadow-2xl ring-1 ring-border/50"
                                                />
                                            ) : (
                                                <img 
                                                    src={selectedEpisode.assets.find(a => a.asset_type === 'image')?.s3_path} 
                                                    className="max-w-full max-h-full object-contain rounded-lg shadow-2xl ring-1 ring-border/50"
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
                        </ResizablePanel>

                        <ResizableHandle withHandle />

                        {/* Bottom: Artifact view */}
                        <ResizablePanel defaultSize={50} minSize={20}>
                            <div className="h-full flex-1 overflow-hidden">
                                <ArtifactView 
                                  plan={selectedEpisode?.plan}
                                  assets={selectedEpisode?.assets}
                                  isConnected={isConnected}
                                />
                            </div>
                        </ResizablePanel>
                    </ResizablePanelGroup>
                </ResizablePanel>
            </ResizablePanelGroup>
        </div>
    </div>
  );
}
