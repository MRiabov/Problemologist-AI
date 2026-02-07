import { useEffect, useState } from 'react';
import { useEpisodes } from '../context/EpisodeContext';
import { fetchEpisodes, runSimulation, type Episode } from '../api/client';
import { 
  Play, 
  Cpu, 
  CircleDot,
  Box,
  BrainCircuit,
  ShieldCheck,
  Check
} from "lucide-react";
import { Button } from "../components/ui/button";
import { Badge } from "../components/ui/badge";
import { cn } from "../lib/utils";
import ReasoningTraces from '../components/workspace/ReasoningTraces';
import ArtifactView from '../components/workspace/ArtifactView';

export default function BenchmarkGeneration() {
  const { 
    selectedEpisode 
  } = useEpisodes();
  const [, setEpisodes] = useState<Episode[]>([]);
  const [simulating, setSimulating] = useState(false);
  const [activeTab, setActiveTab] = useState<'traces' | 'audit'>('traces');

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
            <h2 className="text-lg font-bold tracking-tight">Benchmark Pipeline</h2>
            <div className="flex items-center gap-2">
              <span className="text-[10px] font-black uppercase tracking-widest text-muted-foreground opacity-70">Mechanical Physics Validation</span>
              <Badge variant="outline" className="text-[9px] h-4 py-0 font-mono border-green-500/30 text-green-500 bg-green-500/5">Isolated</Badge>
            </div>
          </div>
        </div>
        
        <div className="flex items-center gap-3">
          <Button 
            onClick={handleRunSimulation}
            disabled={simulating}
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
              journal={selectedEpisode?.journal}
              activeTab={activeTab}
              onTabChange={setActiveTab}
              latestInsight={selectedEpisode?.metadata_vars?.latest_insight}
              isRunning={simulating}
            />
        </div>

        {/* Rightmost Column - span 6/9 */}
        <div className="col-span-6 flex flex-col overflow-hidden">
            {/* Top: Stepper + Viewport */}
            <div className="h-1/2 flex flex-col overflow-hidden border-b">
                {/* Pipeline Status Stepper (Internal) */}
                <div className="bg-muted/10 px-6 py-4 border-b">
                    <div className="flex justify-between items-center max-w-2xl mx-auto">
                        {[
                            { step: 1, label: "Intent", icon: Check, active: true },
                            { step: 2, label: "Strategy", icon: Check, active: true },
                            { step: 3, label: "Synthesis", icon: BrainCircuit, active: true, current: true },
                            { step: 4, label: "Validation", icon: ShieldCheck, active: false }
                        ].map((s) => (
                            <div key={s.step} className="flex flex-col items-center gap-1.5 opacity-80 scale-90">
                            <div className={cn(
                                "w-7 h-7 rounded-full flex items-center justify-center border-2 transition-all",
                                s.active ? "bg-primary text-primary-foreground border-primary" : "bg-muted text-muted-foreground border-transparent",
                                s.current && "ring-2 ring-primary ring-offset-2 scale-110"
                            )}>
                                {s.active && s.step < 3 ? <Check className="h-3 w-3 stroke-[3px]" /> : <s.icon className="h-3 w-3" />}
                            </div>
                            <span className={cn(
                                "text-[8px] font-black uppercase tracking-widest",
                                s.active ? "text-primary" : "text-muted-foreground"
                            )}>
                                {s.label}
                            </span>
                            </div>
                        ))}
                    </div>
                </div>

                {/* Viewport Render Output */}
                <div className="flex-1 relative bg-gradient-to-b from-muted to-background flex items-center justify-center p-8 overflow-hidden">
                    {selectedEpisode?.assets && selectedEpisode.assets.filter(a => a.asset_type === 'video' || a.asset_type === 'image').length > 0 ? (
                        <div className="w-full h-full flex items-center justify-center relative">
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
                        <div className="flex flex-col items-center gap-4 text-muted-foreground/30">
                            <Box className="h-16 w-16" />
                            <span className="text-[10px] font-black uppercase tracking-[0.3em]">No Simulation Trace</span>
                        </div>
                    )}
                </div>
            </div>

            {/* Bottom: Artifacts (MJCF / Validation) */}
            <div className="h-1/2 overflow-hidden flex flex-col">
                <ArtifactView 
                    mjcf={selectedEpisode?.assets?.find(a => a.asset_type === 'mjcf')?.s3_path}
                    validationResults={{
                        integrity_checks: [
                            { label: "XML Schema", status: "success", info: "mj_v3.1" },
                            { label: "Frame Stability", status: "success", info: "ΔE < 1e-4" },
                            { label: "Intersections", status: "active", info: "Verifying..." }
                        ]
                    }}
                />
            </div>
        </div>
      </main>
    </div>
  );
}