import { useEffect, useState } from 'react';
import { fetchEpisodes, fetchSkills, runAgent, fetchEpisode, type Episode, type Skill } from '../api/client';
import { 
  Search, 
  Rocket, 
  CheckCircle2, 
  XCircle, 
  Clock, 
  Code2, 
  GraduationCap, 
  Architecture, 
  Terminal, 
  MessageSquare,
  View3d,
  Box,
  Cpu,
  Zap,
  Play
} from "lucide-react";
import { Button } from "../components/ui/button";
import { Input } from "../components/ui/input";
import { ScrollArea } from "../components/ui/scroll-area";
import { Card, CardContent, CardHeader, CardTitle } from "../components/ui/card";
import { Separator } from "../components/ui/separator";
import { Badge } from "../components/ui/badge";
import { cn } from "../lib/utils";

export default function EngineerWorkspace() {
  const [episodes, setEpisodes] = useState<Episode[]>([]);
  const [skills, setSkills] = useState<Skill[]>([]);
  const [loading, setLoading] = useState(true);
  const [taskPrompt, setTaskPrompt] = useState('');
  const [running, setRunning] = useState(false);
  const [selectedEpisode, setSelectedEpisode] = useState<Episode | null>(null);

  useEffect(() => {
    async function loadData() {
        try {
            const [episodesData, skillsData] = await Promise.all([
                fetchEpisodes(),
                fetchSkills()
            ]);
            setEpisodes(episodesData);
            setSkills(skillsData);
        } catch (e) {
            console.error("Failed to load data", e);
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

  const handleRunAgent = async () => {
    if (!taskPrompt) return;
    setRunning(true);
    try {
        const sessionId = `sess-${Math.random().toString(36).substring(2, 10)}`;
        await runAgent(taskPrompt, sessionId);
        const episodesData = await fetchEpisodes();
        setEpisodes(episodesData);
        setTaskPrompt('');
    } catch (e) {
        console.error("Failed to run agent", e);
    } finally {
        setRunning(false);
    }
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
                    <span className="text-[10px] font-mono font-bold text-green-500 tracking-wider uppercase">Isolated Sandbox</span>
                </div>
            </div>
            
            <div className="flex items-center gap-3">
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

        {/* Main Workspace Layout */}
        <div className="flex flex-1 overflow-hidden">
            {/* Left Panel: History & Skills */}
            <aside className="w-80 flex flex-col border-r bg-muted/10">
                <div className="flex-1 flex flex-col min-h-0 overflow-hidden">
                    <div className="p-4 border-b space-y-4">
                        <div className="flex items-center justify-between">
                            <h3 className="text-[10px] font-black uppercase tracking-widest text-muted-foreground">Episode History</h3>
                            <History className="h-3 w-3 text-muted-foreground" />
                        </div>
                        <div className="relative">
                            <Search className="absolute left-2 top-2.5 h-3.5 w-3.5 text-muted-foreground" />
                            <Input className="h-8 pl-8 text-xs bg-muted/50" placeholder="Filter episodes..." />
                        </div>
                    </div>
                    <ScrollArea className="flex-1">
                        <div className="p-2 space-y-1">
                            {loading ? (
                                 <div className="p-4 text-xs text-muted-foreground text-center">Loading episodes...</div>
                            ) : episodes.length === 0 ? (
                                 <div className="p-4 text-xs text-muted-foreground text-center">No episodes found.</div>
                            ) : (
                                episodes.map(ep => (
                                    <button 
                                        key={ep.id} 
                                        onClick={() => handleSelectEpisode(ep)}
                                        className={cn(
                                          "w-full text-left p-3 rounded-md transition-all group border border-transparent",
                                          selectedEpisode?.id === ep.id 
                                            ? "bg-primary/10 border-primary/20" 
                                            : "hover:bg-muted/50"
                                        )}
                                    >
                                        <div className="flex justify-between items-start mb-1">
                                            <span className={cn(
                                              "text-xs font-semibold truncate flex-1 pr-2",
                                              selectedEpisode?.id === ep.id ? "text-primary" : "text-foreground"
                                            )}>
                                              {ep.task}
                                            </span>
                                            {ep.status === 'running' ? (
                                                <div className="h-2 w-2 rounded-full bg-primary animate-pulse mt-1" />
                                            ) : ep.status === 'completed' || ep.status === 'success' ? (
                                                <CheckCircle2 className="h-3.5 w-3.5 text-green-500" />
                                            ) : (
                                                <XCircle className="h-3.5 w-3.5 text-destructive" />
                                            )}
                                        </div>
                                        <div className="flex items-center gap-2 text-[10px] text-muted-foreground">
                                            <Clock className="h-3 w-3" />
                                            <span>{new Date(ep.created_at).toLocaleTimeString()}</span>
                                            <span>•</span>
                                            <span className="font-mono">{ep.id.substring(0,8)}</span>
                                        </div>
                                    </button>
                                ))
                            )}
                        </div>
                    </ScrollArea>
                </div>
                
                <Separator />
                
                <div className="h-72 flex flex-col bg-muted/20">
                    <div className="p-3 border-b flex items-center justify-between bg-card/50">
                        <h3 className="text-[10px] font-black uppercase tracking-widest text-primary">Learned Skills</h3>
                        <GraduationCap className="h-4 w-4 text-primary" />
                    </div>
                    <ScrollArea className="flex-1">
                        <div className="p-2 space-y-1">
                            {loading ? (
                                <div className="p-4 text-xs text-muted-foreground text-center italic">Syncing skills...</div>
                            ) : (
                                skills.map(skill => (
                                    <div key={skill.name} className="flex items-center gap-3 p-2.5 rounded-md hover:bg-primary/5 cursor-pointer border border-transparent hover:border-primary/10 group transition-all">
                                        <div className="p-1.5 rounded bg-blue-500/10 text-blue-500">
                                          <Code2 className="h-3.5 w-3.5" />
                                        </div>
                                        <span className="text-xs font-medium text-muted-foreground group-hover:text-foreground">{skill.name}</span>
                                    </div>
                                ))
                            )}
                        </div>
                    </ScrollArea>
                </div>
            </aside>

            {/* Middle: Architect & Editor */}
            <main className="flex-1 flex flex-col min-w-0 bg-background overflow-hidden">
                {/* TODO List Section */}
                <section className="h-[40%] flex flex-col border-b">
                    <div className="px-4 py-2 border-b flex items-center justify-between bg-muted/5">
                        <div className="flex items-center gap-2">
                            <Rocket className="h-4 w-4 text-primary" />
                            <span className="text-[10px] font-black uppercase tracking-widest text-muted-foreground">Architect's TODO List</span>
                        </div>
                        {selectedEpisode && (
                            <Badge variant="outline" className="text-[9px] font-mono border-primary/20 text-primary bg-primary/5 px-2">
                                {selectedEpisode.id.substring(0, 12)}
                            </Badge>
                        )}
                    </div>
                    <ScrollArea className="flex-1 p-4">
                        <div className="space-y-3">
                            <div className="flex items-center gap-4 bg-green-500/5 p-3 rounded-lg border border-green-500/10">
                                <CheckCircle2 className="h-5 w-5 text-green-500" />
                                <div className="flex-1">
                                    <div className="text-xs font-bold">Define bounding volume and mounting holes</div>
                                    <div className="text-[10px] text-green-600/80 font-mono mt-0.5 tracking-tight">Status: Verified & Validated</div>
                                </div>
                            </div>
                            <div className="flex items-center gap-4 bg-primary/5 p-3 rounded-lg border border-primary/20">
                                <Clock className="h-5 w-5 text-primary animate-pulse" />
                                <div className="flex-1">
                                    <div className="text-xs font-bold">Implement stress-relieving fillets on inner corners</div>
                                    <div className="text-[10px] text-primary/80 font-mono mt-0.5 tracking-tight">Status: In Progress...</div>
                                </div>
                            </div>
                            <div className="flex items-center gap-4 bg-muted/5 p-3 rounded-lg border border-border opacity-60">
                                <div className="h-5 w-5 rounded-sm border-2 border-muted-foreground" />
                                <div className="flex-1">
                                    <div className="text-xs font-bold">Optimise mass for 3D printing (hollowing)</div>
                                    <div className="text-[10px] text-muted-foreground font-mono mt-0.5 tracking-tight">Status: Queued</div>
                                </div>
                            </div>
                        </div>
                    </ScrollArea>
                </section>

                {/* Editor Section */}
                <section className="flex-1 flex flex-col min-h-0">
                    <div className="flex items-center h-10 bg-muted/20 border-b px-2 gap-1">
                        <Button variant="ghost" size="sm" className="h-8 text-[11px] font-bold border-b-2 border-primary rounded-none px-4 gap-2">
                            <Terminal className="h-3.5 w-3.5 text-blue-400" />
                            impl_build123d.py
                        </Button>
                        <Button variant="ghost" size="sm" className="h-8 text-[11px] font-medium text-muted-foreground rounded-none px-4 hover:bg-muted/50">
                            parameters.yaml
                        </Button>
                    </div>
                    <div className="flex-1 flex font-mono text-sm overflow-hidden bg-[#0d1116]">
                        <div className="w-12 flex-shrink-0 bg-muted/5 text-gray-600 text-right pr-4 pt-4 select-none border-r border-white/5 leading-7 text-[11px]">
                            1<br/>2<br/>3<br/>4<br/>5<br/>6<br/>7<br/>8<br/>9<br/>10<br/>11<br/>12
                        </div>
                        <ScrollArea className="flex-1">
                          <div className="pl-6 pt-4 text-gray-300 leading-7 text-[13px]">
                              <div><span className="syntax-keyword">from</span> build123d <span className="syntax-keyword">import</span> *</div>
                              <div><span className="syntax-keyword">with</span> <span className="syntax-class">BuildPart</span>() <span className="syntax-keyword">as</span> bracket:</div>
                              <div className="pl-6"> <span className="syntax-keyword">with</span> <span className="syntax-class">BuildSketch</span>() <span className="syntax-keyword">as</span> sk:</div>
                              <div className="pl-12"> <span className="syntax-class">Rectangle</span>(width=60, height=80)</div>
                              <div className="pl-12"> <span className="syntax-func">fillet</span>(sk.vertices(), radius=5)</div>
                              <div className="pl-6"> <span className="syntax-func">extrude</span>(amount=10)</div>
                              <br/>
                              <div className="pl-6"><span className="syntax-comment"># Recovered geometry pattern</span></div>
                              <div className="pl-6 relative bg-primary/10 -mx-6 px-6 border-l-2 border-primary py-0.5 my-1">
                                  <span className="absolute right-4 top-1 text-[8px] text-primary font-black uppercase tracking-widest bg-primary/20 px-1.5 rounded">Skill Match</span>
                                  <span className="syntax-func">add_mounting_holes</span>(diameter=6.5, pattern=<span className="syntax-string">"grid"</span>)
                              </div>
                              <div className="pl-6"> <span className="syntax-comment"># Running verification...</span></div>
                              <div className="pl-6"> <span className="syntax-keyword">return</span> bracket</div>
                          </div>
                        </ScrollArea>
                    </div>
                </section>
            </main>

            {/* Right Panel: Assets & Logs */}
            <aside className="w-[450px] flex flex-col border-l">
                {/* 3D/Media Preview */}
                <div className="h-[50%] relative bg-gradient-to-b from-muted/50 to-background flex flex-col overflow-hidden">
                    <div className="absolute top-4 right-4 z-10 w-40 p-3 space-y-2 bg-background/80 backdrop-blur-md rounded-lg border shadow-sm">
                        <div className="flex justify-between items-center">
                            <span className="text-[10px] text-muted-foreground uppercase font-black">Est. Cost</span>
                            <span className="text-xs font-mono font-bold">$4.82</span>
                        </div>
                        <Separator className="opacity-50" />
                        <div className="flex justify-between items-center">
                            <span className="text-[10px] text-muted-foreground uppercase font-black">Weight</span>
                            <span className="text-xs font-mono font-bold">240g</span>
                        </div>
                        <Separator className="opacity-50" />
                        <div className="flex justify-between items-center">
                            <span className="text-[10px] text-muted-foreground uppercase font-black">DFM Score</span>
                            <span className="text-xs font-mono font-bold text-green-500">92%</span>
                        </div>
                    </div>
                    
                    <div className="absolute bottom-4 left-4 z-10 flex gap-2">
                        <Button variant="secondary" size="icon" className="h-8 w-8 rounded-full shadow-lg border-primary/20">
                          <Box className="h-4 w-4" />
                        </Button>
                        <Button variant="secondary" size="icon" className="h-8 w-8 rounded-full shadow-lg">
                          <Search className="h-4 w-4" />
                        </Button>
                    </div>

                    <div className="flex-1 flex items-center justify-center overflow-hidden p-6">
                        {selectedEpisode?.assets && selectedEpisode.assets.filter(a => a.asset_type === 'video' || a.asset_type === 'image').length > 0 ? (
                            <div className="w-full h-full flex items-center justify-center">
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
                            <div className="relative w-64 h-48 border border-dashed border-primary/20 rounded-2xl flex items-center justify-center">
                                <div className="absolute inset-0 bg-primary/5 blur-3xl rounded-full"></div>
                                <div className="flex flex-col items-center gap-3">
                                  <Box className="h-12 w-12 text-primary/30" />
                                  <span className="text-[10px] font-bold text-primary/50 uppercase tracking-widest">No Model Rendered</span>
                                </div>
                            </div>
                        )}
                    </div>
                </div>

                {/* Console / Logs */}
                <div className="flex-1 flex flex-col border-t bg-card/30">
                    <div className="flex h-10 border-b items-center px-4 justify-between bg-muted/10">
                        <div className="flex gap-4">
                          <button className="text-[10px] font-black uppercase tracking-widest text-foreground border-b-2 border-primary h-10">Container Logs</button>
                          <button className="text-[10px] font-black uppercase tracking-widest text-muted-foreground hover:text-foreground h-10">Critic Audit</button>
                        </div>
                        <div className="flex items-center gap-1.5">
                          <div className="h-1.5 w-1.5 rounded-full bg-green-500 animate-pulse" />
                          <span className="text-[9px] font-mono text-muted-foreground uppercase tracking-wider">Live Trace</span>
                        </div>
                    </div>
                    <ScrollArea className="flex-1 bg-black/20">
                        <div className="p-4 font-mono text-[11px] leading-relaxed">
                            {selectedEpisode?.traces && selectedEpisode.traces.length > 0 ? (
                                selectedEpisode.traces.map(trace => (
                                    <div key={trace.id} className="space-y-1 mb-3 border-b border-white/5 pb-2">
                                        <div className="flex justify-between text-[9px] text-muted-foreground">
                                            <span className="text-primary/70">[{new Date(trace.created_at).toLocaleTimeString()}]</span>
                                            <span className="opacity-50 uppercase">Trace: {trace.id.substring(0,8)}</span>
                                        </div>
                                        <div className="text-muted-foreground break-all opacity-90">
                                            {typeof trace.raw_trace === 'string' 
                                                ? trace.raw_trace 
                                                : JSON.stringify(trace.raw_trace, null, 2)}
                                        </div>
                                    </div>
                                ))
                            ) : (
                                <div className="space-y-1">
                                    <div className="text-muted-foreground/50 opacity-80">[0.00s] Initializing build123d kernel...</div>
                                    <div className="text-muted-foreground/50 opacity-80">[0.02s] Loading geometry policy: <span className="text-blue-400">STRICT_PHYSICS</span></div>
                                    <div className="text-blue-300 font-bold">&gt;&gt;&gt; Running impl_build123d.py</div>
                                    <div className="text-green-500/80">[OK] BuildPart created. Vol: 12.4cm³</div>
                                    <div className="text-green-500/80">[OK] Vertex fillets applied. Max Curvature: 0.2</div>
                                    <div className="text-yellow-400/80">[WARN] Hole clearance near edge &lt; 2.0mm</div>
                                    <div className="text-primary animate-pulse">_</div>
                                </div>
                            )}
                        </div>
                    </ScrollArea>
                    
                    {/* Critic Overlay/Bottom Bar */}
                    <div className="p-4 bg-primary/5 border-t">
                        <div className="flex gap-4">
                            <div className="h-10 w-10 rounded-lg bg-primary/10 border border-primary/20 flex items-center justify-center shrink-0">
                                <MessageSquare className="h-5 w-5 text-primary" />
                            </div>
                            <div className="space-y-1">
                                <h4 className="text-[10px] font-black text-primary uppercase tracking-widest">Latest Insight</h4>
                                <p className="text-[11px] text-muted-foreground font-medium leading-normal">
                                  "Geometry is valid, but consider increasing fillet radius on the top flange to reduce stress concentrations identified in simulation."
                                </p>
                            </div>
                        </div>
                    </div>
                </div>
            </aside>
        </div>

        {/* Workspace Footer */}
        <footer className="h-7 shrink-0 bg-muted/30 border-t px-4 flex items-center justify-between text-[10px] text-muted-foreground font-mono">
            <div className="flex items-center gap-6">
                <div className="flex items-center gap-2">
                  <div className="h-1.5 w-1.5 rounded-full bg-green-500 shadow-[0_0_8px_rgba(34,197,94,0.5)]"></div>
                  <span className="font-bold text-foreground/70 uppercase tracking-tight">System Status: Idle</span>
                </div>
                <span>|</span>
                <div className="flex items-center gap-1.5">
                  <Cpu className="h-3 w-3" />
                  <span>Worker: Isolated-01</span>
                </div>
                <span>|</span>
                <div className="flex items-center gap-1.5">
                  <Zap className="h-3 w-3" />
                  <span>Latency: 42ms</span>
                </div>
            </div>
            <div className="flex items-center gap-6">
                <span className="flex items-center gap-1.5">
                  <Terminal className="h-3 w-3" />
                  Python 3.12.3
                </span>
                <span className="bg-primary/10 text-primary px-2 py-0.5 rounded font-bold border border-primary/10">
                  build123d v0.4.0
                </span>
            </div>
        </footer>
    </div>
  );
}