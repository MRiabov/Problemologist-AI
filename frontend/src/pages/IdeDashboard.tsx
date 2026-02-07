import { 
  Search, 
  Play, 
  Code, 
  History, 
  Terminal, 
  Psychology, 
  Settings, 
  FileJson, 
  FileCode, 
  FileSettings, 
  ChevronRight, 
  CheckCircle2, 
  AlertCircle,
  Rotate3d,
  Maximize2,
  Minimize2,
  Grid3X3,
  Bolt,
  DollarSign,
  Activity
} from "lucide-react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { ScrollArea } from "@/components/ui/scroll-area";
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card";
import { Separator } from "@/components/ui/separator";
import { Badge } from "@/components/ui/badge";
import { cn } from "@/lib/utils";

export default function IdeDashboard() {
  return (
    <div className="flex flex-col h-full overflow-hidden bg-background">
      {/* IDE Header */}
      <header className="flex shrink-0 items-center justify-between border-b px-4 h-12 bg-card/80 backdrop-blur-sm z-10">
        <div className="flex items-center gap-4">
          <div className="flex items-center gap-2">
            <div className="flex h-6 w-6 items-center justify-center rounded bg-primary text-primary-foreground">
              <Code className="h-4 w-4" />
            </div>
            <h2 className="text-sm font-bold tracking-tight">CAD-Agent IDE</h2>
            <Badge variant="secondary" className="text-[9px] px-1.5 py-0 h-4 font-normal opacity-70">v3.0.1-beta</Badge>
          </div>
        </div>

        <div className="flex-1 flex justify-center max-w-xl mx-auto">
          <div className="relative w-full max-w-md">
            <Search className="absolute left-2 top-2 h-3.5 w-3.5 text-muted-foreground" />
            <Input 
              className="h-7.5 pl-8 bg-muted/50 border-none focus-visible:ring-1 focus-visible:ring-primary text-xs" 
              placeholder="Search files, symbols, or agents..." 
            />
            <div className="absolute right-2 top-1.5 hidden sm:flex items-center gap-1 opacity-50">
              <kbd className="text-[9px] bg-muted px-1.5 py-0.5 rounded border">⌘</kbd>
              <kbd className="text-[9px] bg-muted px-1.5 py-0.5 rounded border">K</kbd>
            </div>
          </div>
        </div>

        <div className="flex items-center gap-4">
          <nav className="hidden md:flex items-center gap-4 mr-2">
            <Button variant="link" size="sm" className="h-8 text-xs text-muted-foreground hover:text-foreground">File</Button>
            <Button variant="link" size="sm" className="h-8 text-xs text-muted-foreground hover:text-foreground">View</Button>
            <Button variant="link" size="sm" className="h-8 text-xs text-muted-foreground hover:text-foreground">Help</Button>
          </nav>
          <Button size="sm" className="h-8 gap-1.5 px-3">
            <Play className="h-3.5 w-3.5 fill-current" />
            Run
          </Button>
        </div>
      </header>

      {/* Main Content Area */}
      <div className="flex flex-1 overflow-hidden">
        {/* Left Sidebar: Navigation & History */}
        <aside className="w-64 flex flex-col border-r bg-muted/5">
          <div className="p-4 border-b space-y-4">
            <div className="flex items-center justify-between">
              <h3 className="text-[10px] font-black uppercase tracking-widest text-muted-foreground">Execution History</h3>
              <Button variant="ghost" size="icon" className="h-5 w-5 opacity-60">
                <Settings className="h-3 w-3" />
              </Button>
            </div>
            <div className="relative">
              <Search className="absolute left-2 top-2 h-3 w-3 text-muted-foreground" />
              <Input className="h-7 pl-7 text-[10px] bg-muted/50 border-none" placeholder="Filter..." />
            </div>
          </div>
          
          <ScrollArea className="flex-1">
            <div className="space-y-0.5 p-1">
              <div className="p-2.5 rounded-md bg-primary/10 border-l-2 border-primary group cursor-pointer transition-all">
                <div className="flex justify-between items-start mb-1">
                  <span className="text-xs font-bold text-foreground">Ep-1025: Gear Assembly</span>
                  <div className="flex items-center gap-1.5">
                    <div className="h-1.5 w-1.5 rounded-full bg-primary animate-pulse" />
                    <span className="text-[9px] font-black text-primary uppercase">Active</span>
                  </div>
                </div>
                <div className="flex justify-between items-center mt-1">
                  <span className="text-[9px] text-muted-foreground italic">Simulating stability...</span>
                  <Badge variant="outline" className="text-[8px] h-3.5 px-1 opacity-50">Kinematic</Badge>
                </div>
              </div>

              {[1024, 1023, 1022].map(id => (
                <div key={id} className="p-2.5 rounded-md hover:bg-muted/50 border-l-2 border-transparent group cursor-pointer transition-all">
                  <div className="flex justify-between items-start mb-1">
                    <span className="text-xs font-medium text-muted-foreground group-hover:text-foreground">Ep-{id}: {id === 1024 ? 'Bracket Opt' : 'Linkage v2'}</span>
                    <CheckCircle2 className="h-3.5 w-3.5 text-green-500 opacity-60 group-hover:opacity-100" />
                  </div>
                  <div className="flex justify-between items-center">
                    <span className="text-[9px] text-muted-foreground/60">{id === 1024 ? '12m ago' : '1h ago'}</span>
                    <Badge variant="outline" className="text-[8px] h-3.5 px-1 opacity-30">Spatial</Badge>
                  </div>
                </div>
              ))}
            </div>
          </ScrollArea>
        </aside>

        {/* Center: Editor & Thoughts */}
        <main className="flex-1 flex flex-col min-w-0 bg-background overflow-hidden border-r">
          {/* Editor Tabs */}
          <div className="flex items-center h-9 bg-muted/10 border-b px-1">
            <div className="flex items-center gap-2 px-4 h-full bg-background border-t border-x border-t-primary text-[11px] font-bold">
              <FileCode className="h-3.5 w-3.5 text-blue-400" />
              generator.py
            </div>
            <div className="flex items-center gap-2 px-4 h-full text-muted-foreground text-[11px] font-medium hover:bg-muted/30 cursor-pointer border-r border-white/5">
              <FileJson className="h-3.5 w-3.5 text-yellow-500" />
              specs.json
            </div>
            <div className="flex items-center gap-2 px-4 h-full text-muted-foreground text-[11px] font-medium hover:bg-muted/30 cursor-pointer border-r border-white/5">
              <FileSettings className="h-3.5 w-3.5 text-purple-400" />
              agent_config.yaml
            </div>
          </div>

          {/* Editor Area */}
          <div className="flex-1 bg-[#0d1116] font-mono text-[12px] overflow-hidden flex">
            <div className="w-10 shrink-0 bg-muted/5 border-r border-white/5 text-muted-foreground/30 text-right pr-3 pt-4 select-none leading-6">
              1<br/>2<br/>3<br/>4<br/>5<br/>6<br/>7<br/>8<br/>9<br/>10<br/>11<br/>12
            </div>
            <ScrollArea className="flex-1">
              <div className="p-4 pl-6 text-gray-300 leading-6">
                <div><span className="syntax-keyword">import</span> cad_library <span className="syntax-keyword">as</span> cad</div>
                <div><span className="syntax-keyword">from</span> agents.physics <span className="syntax-keyword">import</span> ConstraintSolver</div>
                <br/>
                <div><span className="syntax-comment"># Assembly optimization kernel</span></div>
                <div><span className="syntax-keyword">def</span> <span className="syntax-func">optimize_bracket_structure</span>(params):</div>
                <div className="pl-6">solver = <span className="syntax-class">ConstraintSolver</span>(tolerance=1e-5)</div>
                <div className="pl-6">base_plate = cad.<span className="syntax-func">create_box</span>(width=params[<span className="syntax-string">'w'</span>], height=10)</div>
                <br/>
                <div className="relative bg-primary/10 -mx-6 px-6 border-l-2 border-primary py-0.5 my-1">
                  <span className="absolute right-4 top-1 text-[8px] font-black uppercase text-primary/50 tracking-tighter">LLM Generating...</span>
                  <div className="pl-6"><span className="syntax-keyword">if</span> solver.<span className="syntax-func">check_collision</span>(base_plate):</div>
                  <div className="pl-12">params[<span className="syntax-string">'w'</span>] += 2.5 <span className="syntax-comment"># Resolve interference</span></div>
                </div>
                <div className="pl-6"><span className="syntax-keyword">return</span> base_plate</div>
              </div>
            </ScrollArea>
          </div>

          {/* Agent Cognition Panel */}
          <div className="h-[35%] flex flex-col border-t bg-card/30">
            <div className="px-4 py-2 border-b flex items-center justify-between bg-muted/20 backdrop-blur-sm">
              <div className="flex items-center gap-2">
                <div className="p-1 rounded bg-purple-500/10 text-purple-500">
                  <Bolt className="h-3.5 w-3.5" />
                </div>
                <span className="text-[10px] font-black uppercase tracking-widest text-muted-foreground">Agent Cognition Stream</span>
              </div>
              <Badge variant="outline" className="bg-green-500/10 text-green-500 border-green-500/20 text-[9px] h-4">THINKING...</Badge>
            </div>
            <ScrollArea className="flex-1 p-4">
              <div className="space-y-6">
                <div className="flex gap-4 opacity-50">
                  <div className="flex flex-col items-center shrink-0">
                    <div className="h-6 w-6 rounded-full border-2 border-muted-foreground/30 flex items-center justify-center">
                      <ChevronRight className="h-3 w-3" />
                    </div>
                    <div className="w-px flex-1 bg-border my-1" />
                  </div>
                  <div className="space-y-2">
                    <h4 className="text-[10px] font-bold text-muted-foreground uppercase tracking-tight">Visual Analysis</h4>
                    <div className="text-xs text-muted-foreground leading-relaxed bg-muted/5 p-3 rounded-lg border">
                      Detected 2 potential collision points on the Z-axis extrusion during kinematic sweep.
                    </div>
                  </div>
                </div>

                <div className="flex gap-4">
                  <div className="flex flex-col items-center shrink-0">
                    <div className="h-8 w-8 rounded-full border-2 border-purple-500/40 bg-purple-500/5 flex items-center justify-center shadow-[0_0_15px_rgba(168,85,247,0.1)]">
                      <Bolt className="h-4 w-4 text-purple-500" />
                    </div>
                    <div className="w-px flex-1 bg-purple-500/20 my-1" />
                  </div>
                  <div className="space-y-2 flex-1">
                    <h4 className="text-[10px] font-bold text-purple-500 uppercase tracking-tight">Reasoning</h4>
                    <Card className="bg-[#1e1e2e]/50 border-purple-500/20 shadow-xl shadow-purple-500/5">
                      <CardContent className="p-3 text-xs leading-relaxed text-foreground">
                        The collision implies the bracket is too narrow for the bearing housing. I will increase the <code className="bg-purple-500/20 px-1 rounded text-purple-400">width</code> parameter by 2.5mm and re-trigger the solver.
                      </CardContent>
                    </Card>
                  </div>
                </div>
              </div>
            </ScrollArea>
          </div>
        </main>

        {/* Right Sidebar: 3D & Console */}
        <aside className="w-96 flex flex-col bg-muted/5">
          {/* Viewport Area */}
          <div className="h-[55%] relative flex flex-col overflow-hidden bg-gradient-to-b from-card to-background border-b">
            {/* Viewport Overlay Controls */}
            <div className="absolute top-4 left-4 z-10 flex flex-col gap-1.5 p-1 bg-background/80 backdrop-blur-md rounded border shadow-lg">
              <Button variant="ghost" size="icon" className="h-7 w-7"><Rotate3d className="h-3.5 w-3.5" /></Button>
              <Button variant="ghost" size="icon" className="h-7 w-7"><Maximize2 className="h-3.5 w-3.5" /></Button>
              <Button variant="ghost" size="icon" className="h-7 w-7"><Grid3X3 className="h-3.5 w-3.5" /></Button>
            </div>

            <div className="absolute top-4 right-4 z-10 w-40 p-3 bg-background/90 backdrop-blur-md rounded border shadow-xl space-y-2.5">
              <div className="flex justify-between items-center">
                <span className="text-[9px] font-black text-muted-foreground uppercase flex items-center gap-1.5">
                  <Bolt className="h-3 w-3 text-yellow-500" />
                  Energy
                </span>
                <span className="text-xs font-mono font-bold">420kJ</span>
              </div>
              <Separator className="opacity-50" />
              <div className="flex justify-between items-center">
                <span className="text-[9px] font-black text-muted-foreground uppercase flex items-center gap-1.5">
                  <DollarSign className="h-3 w-3 text-green-500" />
                  Cost
                </span>
                <span className="text-xs font-mono font-bold">$0.04</span>
              </div>
              <Separator className="opacity-50" />
              <div className="flex justify-between items-center">
                <span className="text-[9px] font-black text-muted-foreground uppercase flex items-center gap-1.5">
                  <Activity className="h-3 w-3 text-red-500" />
                  Impulse
                </span>
                <span className="text-xs font-mono font-bold">0</span>
              </div>
            </div>

            {/* Placeholder for 3D Viewport Content */}
            <div className="flex-1 flex items-center justify-center p-12">
              <div className="relative group">
                <div className="absolute -inset-10 bg-primary/20 rounded-full blur-3xl group-hover:bg-primary/30 transition-all duration-1000"></div>
                <div className="relative w-40 h-40 border border-primary/30 rounded-full flex items-center justify-center animate-[spin_20s_linear_infinite]">
                  <div className="w-24 h-24 border border-dashed border-primary/50 rounded-full"></div>
                  <div className="absolute w-full h-px bg-primary/20 top-1/2"></div>
                  <div className="absolute h-full w-px bg-primary/20 left-1/2"></div>
                </div>
                <div className="absolute inset-0 flex items-center justify-center">
                  <Card className="bg-background/80 backdrop-blur-sm border-primary/40 shadow-2xl p-4 rotate-12 scale-110">
                    <span className="text-[10px] font-black text-primary uppercase tracking-[0.2em]">Bracket_v2</span>
                  </Card>
                </div>
              </div>
            </div>
          </div>

          {/* Log / Terminal Area */}
          <div className="flex-1 flex flex-col overflow-hidden">
            <div className="px-4 h-9 flex items-center justify-between border-b bg-muted/10">
              <div className="flex items-center gap-2">
                <Terminal className="h-3.5 w-3.5 text-muted-foreground" />
                <span className="text-[10px] font-black uppercase tracking-widest text-muted-foreground">Execution Log</span>
              </div>
              <Badge variant="secondary" className="text-[8px] h-4 font-mono">LIVE</Badge>
            </div>
            <ScrollArea className="flex-1 bg-black/40">
              <div className="p-4 font-mono text-[10px] leading-relaxed space-y-1 text-muted-foreground">
                <div className="text-primary/70">[14:02:22] Simulation environment initialized.</div>
                <div className="text-primary/70">[14:02:23] Loading physics engine (MuJoCo 3.1)...</div>
                <div className="text-purple-400 opacity-80">[AGENT] 14:02:24 Starting geometry generation loop.</div>
                <div className="text-foreground/70">&gt; Generating mesh for 'bracket_v2'... Done (45ms)</div>
                <div className="bg-yellow-500/10 text-yellow-500 p-2 rounded border border-yellow-500/20 my-1">
                  [WARN] 14:02:25 Mesh intersection detected at vertex #402. Tolerance exceeded (0.02mm &gt; 0.01mm).
                </div>
                <div className="text-purple-400 opacity-80">[AGENT] 14:02:25 Correcting transformation...</div>
                <div className="text-foreground/70">&gt; Re-calculating constraints...</div>
                <div className="text-green-500 font-bold mt-2">[SUCCESS] 14:02:26 Optimization complete.</div>
                <div className="animate-pulse text-primary mt-1">_</div>
              </div>
            </ScrollArea>
          </div>
        </aside>
      </div>
    </div>
  );
}