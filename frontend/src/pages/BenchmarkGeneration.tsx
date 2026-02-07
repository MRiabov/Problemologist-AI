import { useEffect, useState } from 'react';
import { fetchEpisodes, fetchEpisode, runSimulation, type Episode } from '../api/client';
import { 
  Play, 
  Database, 
  Check, 
  FileJson as Schema, 
  ShieldCheck, 
  Settings2, 
  History, 
  Cpu, 
  Terminal, 
  Dices, 
  Layers, 
  CircleDot,
  ArrowRight,
  MonitorPlay,
  Box,
  BrainCircuit,
  DatabaseZap,
  CheckCircle2
} from "lucide-react";
import { Button } from "../components/ui/button";
import { ScrollArea } from "../components/ui/scroll-area";
import { Card, CardContent, CardHeader, CardTitle } from "../components/ui/card";
import { Separator } from "../components/ui/separator";
import { Badge } from "../components/ui/badge";
import { cn } from "../lib/utils";

export default function BenchmarkGeneration() {
  const [episodes, setEpisodes] = useState<Episode[]>([]);
  const [selectedEpisode, setSelectedEpisode] = useState<Episode | null>(null);
  const [loading, setLoading] = useState(true);
  const [simulating, setSimulating] = useState(false);

  useEffect(() => {
    async function loadData() {
        try {
            const data = await fetchEpisodes();
            setEpisodes(data);
        } catch (e) {
            console.error("Failed to load episodes", e);
        } finally {
            setLoading(false);
        }
    }
    loadData();
  }, []);

  const handleSelectEpisode = async (ep: Episode) => {
    try {
        const fullEp = await fetchEpisode(ep.id);
        setSelectedEpisode(fullEp);
    } catch (e) {
        console.error("Failed to fetch episode details", e);
        setSelectedEpisode(ep);
    }
  };

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
          <Button variant="outline" size="icon" className="h-10 w-10">
            <Database className="h-4 w-4" />
          </Button>
        </div>
      </header>

      {/* Pipeline Status Stepper */}
      <div className="border-b bg-muted/20 px-6 py-5 shrink-0">
        <div className="max-w-4xl mx-auto flex justify-between items-center relative">
          <div className="absolute left-0 top-[18px] w-full h-0.5 bg-muted -z-0" />
          <div className="absolute left-0 top-[18px] w-2/3 h-0.5 bg-primary -z-0" />

          {[
            { step: 1, label: "Intent", icon: Check, active: true },
            { step: 2, label: "Strategy", icon: Check, active: true },
            { step: 3, label: "Synthesis", icon: BrainCircuit, active: true, current: true },
            { step: 4, label: "Validation", icon: ShieldCheck, active: false }
          ].map((s) => (
            <div key={s.step} className="relative z-10 flex flex-col items-center gap-2">
              <div className={cn(
                "w-9 h-9 rounded-full flex items-center justify-center border-4 border-background transition-all duration-500",
                s.active ? "bg-primary text-primary-foreground shadow-[0_0_15px_rgba(var(--primary),0.3)]" : "bg-muted text-muted-foreground",
                s.current && "ring-2 ring-primary ring-offset-2 ring-offset-background scale-110"
              )}>
                {s.active && s.step < 3 ? <Check className="h-4 w-4 stroke-[3px]" /> : <s.icon className="h-4 w-4" />}
              </div>
              <span className={cn(
                "text-[10px] font-black uppercase tracking-widest",
                s.active ? "text-primary" : "text-muted-foreground"
              )}>
                {s.label}
              </span>
            </div>
          ))}
        </div>
      </div>

      {/* Workspace Area */}
      <main className="flex-1 flex overflow-hidden">
        {/* Left: History Sidebar */}
        <aside className="w-72 flex flex-col border-r bg-muted/5 overflow-hidden">
            <div className="p-4 border-b flex items-center justify-between">
                <h3 className="text-[10px] font-black uppercase tracking-widest text-muted-foreground">Recent Benchmarks</h3>
                <History className="h-3 w-3 text-muted-foreground" />
            </div>
            <ScrollArea className="flex-1">
              <div className="p-2 space-y-1">
                  {loading ? (
                      <div className="p-4 text-xs text-muted-foreground text-center italic">Loading history...</div>
                  ) : (
                      episodes.map(ep => (
                          <button 
                              key={ep.id}
                              onClick={() => handleSelectEpisode(ep)}
                              className={cn(
                                "w-full text-left p-3 rounded-md transition-all group border border-transparent",
                                selectedEpisode?.id === ep.id ? "bg-primary/10 border-primary/10" : "hover:bg-muted/50"
                              )}
                          >
                              <div className={cn(
                                "text-xs font-bold truncate mb-1",
                                selectedEpisode?.id === ep.id ? "text-primary" : "text-foreground"
                              )}>
                                {ep.task || ep.id.substring(0,8)}
                              </div>
                              <div className="flex justify-between items-center text-[9px]">
                                  <span className="text-muted-foreground font-mono">{new Date(ep.created_at).toLocaleTimeString()}</span>
                                  <Badge variant="outline" className={cn(
                                    "text-[8px] h-3.5 px-1 py-0 border-none uppercase font-bold",
                                    ep.status === 'running' ? "bg-primary/20 text-primary animate-pulse" : "bg-green-500/20 text-green-500"
                                  )}>
                                    {ep.status}
                                  </Badge>
                              </div>
                          </button>
                      ))
                  )}
              </div>
            </ScrollArea>
        </aside>

        {/* Middle: Configuration & MJCF */}
        <div className="flex-1 flex flex-col border-r bg-background min-w-[450px]">
          {/* Top: Expected Strategy */}
          <section className="h-1/3 flex flex-col border-b">
            <div className="px-4 py-2 bg-muted/10 border-b flex justify-between items-center">
              <h3 className="text-[10px] font-black uppercase tracking-widest text-muted-foreground flex items-center gap-2">
                <BrainCircuit className="h-3.5 w-3.5 text-primary" />
                Randomization Strategy
              </h3>
              <span className="text-[10px] text-primary/40 font-mono">v1.2.0-stable</span>
            </div>
            <ScrollArea className="flex-1 p-4 bg-black/5">
              <div className="space-y-4 font-mono text-[12px]">
                <div className="bg-primary/5 p-3 rounded border-l-2 border-primary">
                  <span className="text-primary font-bold"># Generation Intent:</span>
                  <p className="mt-2 text-muted-foreground/90 leading-relaxed">
                    Instantiate a randomized pendulum scenario. Assert energy transfer to a hinged target &gt; 2.5J. 
                    Randomize pivot height [2.5m - 3.5m] and mass distribution [0.5kg - 2.0kg].
                  </p>
                </div>
                <div className="space-y-1.5 pl-2 opacity-80">
                  <p><span className="text-yellow-600 font-bold">SET</span> platform_pos = [0, 0, 0]</p>
                  <p><span className="text-yellow-600 font-bold">VARY</span> pivot_z in RANGE(2.5, 3.5)</p>
                  <p><span className="text-yellow-600 font-bold">VERIFY</span> impact_sensor.triggered == TRUE</p>
                </div>
              </div>
            </ScrollArea>
          </section>

          {/* Bottom: XML/MJCF Output */}
          <section className="flex-1 flex flex-col bg-[#080c10]">
            <div className="px-4 py-2 bg-black/40 border-b flex justify-between items-center">
              <h3 className="text-[10px] font-black uppercase tracking-widest text-muted-foreground flex items-center gap-2">
                <Terminal className="h-3.5 w-3.5 text-primary" />
                MuJoCo Benchmark (MJCF)
              </h3>
              <Button variant="ghost" size="sm" className="h-6 text-[9px] gap-1 px-2 uppercase font-bold text-muted-foreground hover:text-foreground">
                <Layers className="h-3 w-3" /> Copy XML
              </Button>
            </div>
            <div className="flex-1 flex overflow-hidden group relative">
              <div className="w-10 shrink-0 bg-black/20 border-r border-white/5 text-muted-foreground/20 text-right pr-3 pt-4 select-none font-mono text-[11px] leading-6">
                1<br/>2<br/>3<br/>4<br/>5<br/>6<br/>7<br/>8<br/>9<br/>10<br/>11<br/>12
              </div>
              <ScrollArea className="flex-1">
                <div className="p-4 text-slate-400 leading-6 font-mono text-[12px]">
                  {selectedEpisode?.assets?.find(a => a.asset_type === 'mjcf') ? (
                      <span className="text-muted-foreground/60 italic opacity-50">
                        &lt;!-- MJCF data linked from: {selectedEpisode.assets.find(a => a.asset_type === 'mjcf')?.s3_path.split('/').pop()} --&gt;
                      </span>
                  ) : (
                      <>
                      <span className="text-blue-400">&lt;mujoco&gt;</span>
                      {"\n"}
                      <span className="text-blue-400">&lt;worldbody&gt;</span>
                      {"\n"}
                      <span className="text-muted-foreground/50 opacity-60">  &lt;!-- Procedural Floor --&gt;</span>
                      {"\n"}
                      <span className="text-blue-300">  &lt;geom</span> name="floor" type="plane" size="5 5 .1" <span className="text-blue-300">/&gt;</span>
                      {"\n"}
                      <span className="text-blue-300">  &lt;body</span> name="pendulum" pos="0 0 3" <span className="text-blue-300">&gt;</span>
                      {"\n"}
                      <span className="text-blue-300">    &lt;joint</span> type="hinge" axis="0 1 0" <span className="text-blue-300">/&gt;</span>
                      {"\n"}
                      <span className="text-blue-300">    &lt;geom</span> size="0.05 1.5" type="cylinder" <span className="text-blue-300">/&gt;</span>
                      {"\n"}
                      <span className="text-blue-300">    &lt;geom</span> pos="0 0 -1.5" size="0.2" type="sphere" rgba=".8 .2 .2 1" <span className="text-blue-300">/&gt;</span>
                      {"\n"}
                      <span className="text-blue-300">  &lt;/body&gt;</span>
                      {"\n"}
                      <span className="text-blue-400">&lt;/worldbody&gt;</span>
                      {"\n"}
                      <span className="text-blue-400">&lt;/mujoco&gt;</span>
                      </>
                  )}
                </div>
              </ScrollArea>
            </div>
          </section>
        </div>

        {/* Right: Viewport & Validation */}
        <div className="w-7/12 flex flex-col">
          {/* Main Preview */}
          <div className="flex-1 relative bg-gradient-to-b from-muted/50 to-background overflow-hidden flex items-center justify-center p-8">
            <div className="absolute top-4 left-4 z-20 flex flex-col gap-2">
              <div className="bg-background/80 backdrop-blur border rounded-lg p-1.5 flex flex-col gap-1 shadow-xl">
                <Button variant="ghost" size="icon" className="h-8 w-8"><MonitorPlay className="h-4 w-4" /></Button>
                <Button variant="ghost" size="icon" className="h-8 w-8"><Dices className="h-4 w-4 text-primary" /></Button>
                <Separator className="mx-1 my-1 opacity-50" />
                <Button variant="ghost" size="icon" className="h-8 w-8"><Settings2 className="h-4 w-4" /></Button>
              </div>
              <Badge variant="secondary" className="font-mono text-[9px] h-5 opacity-60">SEED: 8841-A2</Badge>
            </div>
            
            <div className="absolute top-4 right-4 z-20 text-right space-y-2">
              <div className="bg-background/80 backdrop-blur border rounded-md px-3 py-1.5 shadow-md flex items-center gap-2">
                <span className="text-[10px] font-black uppercase text-muted-foreground">Engine:</span>
                <span className="text-[10px] font-black text-primary uppercase">MuJoCo Native</span>
              </div>
              <Badge variant="outline" className="text-yellow-500 border-yellow-500/30 bg-yellow-500/5 text-[9px] uppercase font-bold tracking-tight">
                Preview Variation 3/10
              </Badge>
            </div>

            {/* Render Output */}
            <div className="w-full h-full flex items-center justify-center">
              {selectedEpisode?.assets && selectedEpisode.assets.filter(a => a.asset_type === 'video' || a.asset_type === 'image').length > 0 ? (
                  <div className="w-full h-full flex items-center justify-center relative">
                    <div className="absolute inset-0 bg-primary/5 blur-3xl rounded-full scale-75 animate-pulse" />
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
                <div className="relative w-[400px] h-[300px] flex items-center justify-center">
                  <div className="absolute inset-0 border border-dashed border-primary/20 rounded-3xl" />
                  <div className="flex flex-col items-center gap-4 text-muted-foreground/30">
                    <Box className="h-16 w-16" />
                    <span className="text-[10px] font-black uppercase tracking-[0.3em]">No Simulation Trace</span>
                  </div>
                </div>
              )}
            </div>
          </div>

          {/* Validation Footer Panel */}
          <div className="bg-card border-t p-6 shadow-inner">
            <div className="flex items-center justify-between gap-8">
              <div className="flex-1 space-y-5">
                <div className="flex items-center gap-2">
                  <ShieldCheck className="h-5 w-5 text-primary" />
                  <h4 className="text-xs font-black uppercase tracking-[0.15em]">MuJoCo Integrity Checklist</h4>
                </div>
                <div className="grid grid-cols-3 gap-4">
                  {[
                    { label: "XML Schema", status: "success", info: "mj_v3.1" },
                    { label: "Frame Stability", status: "success", info: "ΔE < 1e-4" },
                    { label: "Intersections", status: "active", info: "Verifying..." }
                  ].map((check) => (
                    <div key={check.label} className="bg-muted/30 p-3 rounded-lg border border-border/50">
                      <div className="flex items-center justify-between mb-2">
                        <span className="text-[9px] uppercase font-black text-muted-foreground tracking-widest">{check.label}</span>
                        {check.status === 'success' ? (
                          <CheckCircle2 className="h-4 w-4 text-green-500" />
                        ) : (
                          <CircleDot className="h-4 w-4 text-primary animate-spin" />
                        )}
                      </div>
                      <div className="h-1 w-full bg-muted rounded-full overflow-hidden">
                        <div className={cn(
                          "h-full rounded-full transition-all duration-1000",
                          check.status === 'success' ? "bg-green-500 w-full" : "bg-primary w-2/3"
                        )} />
                      </div>
                      <p className="mt-2 text-[10px] text-muted-foreground/60 font-mono tracking-tighter">{check.info}</p>
                    </div>
                  ))}
                </div>
              </div>

              <div className="flex flex-col gap-3 shrink-0">
                <Button className="h-12 px-6 gap-3 group relative overflow-hidden font-bold">
                  <div className="absolute inset-0 bg-gradient-to-r from-primary/0 via-white/10 to-primary/0 translate-x-[-100%] group-hover:translate-x-[100%] transition-transform duration-1000" />
                  <span className="uppercase tracking-widest text-xs">Commit to Benchmark Suite</span>
                  <DatabaseZap className="h-4 w-4 group-hover:scale-110 transition-transform" />
                </Button>
                <div className="flex items-center justify-center gap-2 text-[9px] text-muted-foreground font-bold uppercase tracking-tighter opacity-60">
                  <div className="h-1.5 w-1.5 rounded-full bg-muted-foreground/30" />
                  Target: shared_benchmarks.sqlite
                </div>
              </div>
            </div>
          </div>
        </div>
      </main>
    </div>
  );
}