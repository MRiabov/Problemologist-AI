import { useEffect, useState } from 'react';
import { useEpisodes } from '../context/EpisodeContext';
import { useConnection } from '../context/ConnectionContext';
import { fetchEpisodes, runSimulation, type Episode } from '../api/client';
import { 
  Play, 
  Cpu, 
  CircleDot,
  Signal,
  SignalLow
} from "lucide-react";
import { Button } from "../components/ui/button";
import { Badge } from "../components/ui/badge";
import ReasoningTraces from '../components/workspace/ReasoningTraces';
import ArtifactView from '../components/workspace/ArtifactView';
import ModelViewer from '../components/visualization/ModelViewer';
import ConnectionError from '../components/shared/ConnectionError';

export default function BenchmarkGeneration() {
  const { 
    selectedEpisode,
    running
  } = useEpisodes();
  const { isConnected } = useConnection();
  const [, setEpisodes] = useState<Episode[]>([]);
  const [simulating, setSimulating] = useState(false);

  useEffect(() => {
    async function loadData() {
        try {
            const data = await fetchEpisodes();
            setEpisodes(data);
        } catch (e) {
            console.error("Failed to load episodes", e);
        }
    }
    loadData();
  }, []);

  const handleRunSimulation = async () => {
    setSimulating(true);
    try {
        const sessionId = `sim-${Math.random().toString(36).substring(2, 10)}`;
        await runSimulation(sessionId);
        const data = await fetchEpisodes();
        setEpisodes(data);
    } catch (e) {
        console.error("Failed to run simulation", e);
    } finally {
        setSimulating(false);
    }
  };

  return (
    <div className="flex flex-col h-full overflow-hidden bg-background">
      {/* Page Header */}
      <header className="flex shrink-0 items-center justify-between border-b px-6 h-16 bg-card/50 backdrop-blur-sm">
        <div className="flex items-center gap-4">
          <div className="size-10 flex items-center justify-center bg-primary/10 rounded-lg text-primary border border-primary/20">
            <Cpu className="h-6 w-6" />
          </div>
          <div>
            <div className="flex items-center gap-2">
              <h2 className="text-lg font-bold tracking-tight">Benchmark Pipeline</h2>
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
              <span className="text-[10px] font-black uppercase tracking-widest text-muted-foreground opacity-70">Mechanical Physics Validation</span>
            </div>
          </div>
        </div>
        
        <div className="flex items-center gap-3">
          {!isConnected && (
            <Badge variant="outline" className="text-[9px] h-6 px-3 font-bold border-red-500/30 text-red-500 bg-red-500/5 uppercase tracking-widest animate-pulse">
              System Offline
            </Badge>
          )}
          <Button 
            onClick={handleRunSimulation}
            disabled={simulating || !isConnected}
            className="gap-2 h-10 px-6 font-bold"
          >
            {simulating ? <CircleDot className="h-4 w-4 animate-spin" /> : <Play className="h-4 w-4 fill-current" />}
            {simulating ? 'SIMULATING...' : 'RUN PIPELINE'}
          </Button>
        </div>
      </header>

      {/* Main Content Area (Grid 9-cols relative to Outlet) */}
      <main className="flex-1 grid grid-cols-9 overflow-hidden">
        {/* Middle Column (Traces) - span 3/9 */}
        <div className="col-span-3 border-r overflow-hidden">
            <ReasoningTraces 
              traces={selectedEpisode?.traces}
              isRunning={simulating || running}
              isConnected={isConnected}
            />
        </div>

        {/* Rightmost Column - span 6/9 */}
        <div className="col-span-6 flex flex-col overflow-hidden">
            {/* Top: Viewport */}
            <div className="h-1/2 flex flex-col overflow-hidden border-b relative bg-gradient-to-b from-muted to-background flex items-center justify-center">
                    {selectedEpisode?.assets && selectedEpisode.assets.filter(a => a.asset_type === 'video' || a.asset_type === 'image').length > 0 ? (
                        <div className="w-full h-full flex items-center justify-center relative p-8">
                            {!isConnected && <ConnectionError className="absolute inset-0 z-[60]" />}
                            {selectedEpisode.assets.find(a => a.asset_type === 'video') ? (
                                <video 
                                    src={selectedEpisode.assets.find(a => a.asset_type === 'video')?.s3_path} 
                                    controls 
                                    className="max-w-full max-h-full rounded-xl shadow-2xl border-4 border-card z-10"
                                />
                            ) : (
                                <img 
                                    src={selectedEpisode.assets.find(a => a.asset_type === 'image')?.s3_path} 
                                    className="max-w-full max-h-full object-contain rounded-xl shadow-2xl border-4 border-card z-10"
                                />
                            )}
                        </div>
                    ) : (
                        <div className="w-full h-full relative">
                            <ModelViewer 
                                className="w-full h-full" 
                                isConnected={isConnected}
                            />
                            <div className="absolute top-4 left-1/2 -translate-x-1/2 z-10 pointer-events-none opacity-0 group-hover:opacity-100 transition-opacity">
                                <Badge variant="outline" className="bg-background/50 backdrop-blur-sm text-[10px] uppercase font-bold tracking-widest px-3 py-1 border-primary/20 text-primary/70">
                                    Simulation Preview • Multi-Body
                                </Badge>
                            </div>
                        </div>
                    )}
                </div>

            {/* Bottom: Artifacts (MJCF / Validation) */}
            <div className="h-1/2 overflow-hidden flex flex-col">
                <ArtifactView 
                    plan={selectedEpisode?.plan}
                    assets={selectedEpisode?.assets}
                    isConnected={isConnected}
                />
            </div>
        </div>
      </main>
    </div>
  );
}