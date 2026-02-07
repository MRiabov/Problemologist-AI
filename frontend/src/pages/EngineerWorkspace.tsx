import { useState } from 'react';
import { useEpisodes } from '../context/EpisodeContext';
import { 
  Search, 
  XCircle, 
  Zap,
  Play,
  Box
} from "lucide-react";
import { Button } from "../components/ui/button";
import { Input } from "../components/ui/input";
import { Badge } from "../components/ui/badge";
import ReasoningTraces from '../components/workspace/ReasoningTraces';
import ArtifactView from '../components/workspace/ArtifactView';

import ModelViewer from '../components/visualization/ModelViewer';

export default function EngineerWorkspace() {
  const { 
    selectedEpisode, 
    running, 
    startAgent, 
    setRunning 
  } = useEpisodes();
  
  const [taskPrompt, setTaskPrompt] = useState('');

  const handleRunAgent = async () => {
    if (!taskPrompt) return;
    await startAgent(taskPrompt);
    setTaskPrompt('');
  };

  const handleInterrupt = () => {
    console.log("Interrupting execution...");
    setRunning(false);
  };

  return (
    <div className="flex flex-col h-full overflow-hidden bg-background">
        {/* Workspace Header */}
        <header className="flex shrink-0 items-center justify-between border-b px-4 h-14 bg-card/50 backdrop-blur-sm">
            <div className="flex items-center gap-4 flex-1">
                <div className="relative w-full max-w-xl">
                    <Search className="absolute left-2.5 top-2.5 h-4 w-4 text-muted-foreground" />
                    <Input 
                        className="pl-9 bg-muted/50 border-none focus-visible:ring-1 focus-visible:ring-primary h-9 text-sm" 
                        placeholder="Describe a mechanical task for the engineer..." 
                        type="text"
                        value={taskPrompt}
                        onChange={(e) => setTaskPrompt(e.target.value)}
                        onKeyDown={(e) => e.key === 'Enter' && handleRunAgent()}
                    />
                </div>
                <div className="flex items-center gap-2 px-3 py-1 bg-green-500/10 border border-green-500/20 rounded-full">
                    <span className="relative flex h-2 w-2">
                        <span className="animate-ping absolute inline-flex h-full w-full rounded-full bg-green-400 opacity-75"></span>
                        <span className="relative inline-flex rounded-full h-2 w-2 bg-green-500"></span>
                    </span>
                    <span className="text-[10px] font-black uppercase tracking-widest text-green-500">Live Worker</span>
                </div>
            </div>
            
            <div className="flex items-center gap-3">
                {running && (
                    <Button
                        onClick={handleInterrupt}
                        variant="destructive"
                        size="sm"
                        className="gap-2 h-9"
                    >
                        <XCircle className="h-4 w-4" />
                        INTERRUPT
                    </Button>
                )}
                <Button
                    onClick={handleRunAgent}
                    disabled={running || !taskPrompt}
                    size="sm"
                    className="gap-2 h-9"
                >
                    {running ? (
                      <Zap className="h-4 w-4 animate-pulse" />
                    ) : (
                      <Play className="h-4 w-4" />
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
                  isRunning={running}
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
                        <Button variant="secondary" size="icon" className="h-8 w-8 rounded-full shadow-lg border-primary/20">
                          <Box className="h-4 w-4" />
                        </Button>
                    </div>

                    {/* Render Area */}
                    <div className="w-full h-full flex items-center justify-center overflow-hidden bg-muted/20 relative group">
                        {selectedEpisode?.assets && selectedEpisode.assets.filter(a => a.asset_type === 'video' || a.asset_type === 'image').length > 0 ? (
                            <div className="w-full h-full flex items-center justify-center p-6">
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
                                <ModelViewer className="w-full h-full" />
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
                      code={selectedEpisode?.assets?.find(a => a.asset_type === 'python')?.s3_path}
                      mjcf={selectedEpisode?.assets?.find(a => a.asset_type === 'mjcf')?.s3_path}
                    />
                </div>
            </main>
        </div>
    </div>
  );
}
