import { useEpisodes } from '../context/EpisodeContext';
import { useConnection } from '../context/ConnectionContext';
import { fetchEpisodes, type Episode } from '../api/client';
import { 
  Cpu, 
  SignalLow,
  AlertCircle
} from "lucide-react";
import { Button } from "../components/ui/button";
import { Badge } from "../components/ui/badge";
import ChatWindow from '../components/workspace/ChatWindow';
import ArtifactView from '../components/workspace/ArtifactView';
import { 
  ResizableHandle, 
  ResizablePanel, 
  ResizablePanelGroup 
} from "../components/ui/resizable";
import ModelViewer from '../components/visualization/ModelViewer';
import ConnectionError from '../components/shared/ConnectionError';

export default function BenchmarkGeneration() {
  const { 
    selectedEpisode,
    running
  } = useEpisodes();
  const { isConnected } = useConnection();
  const [, setEpisodes] = useState<Episode[]>([]);
  const [error, setError] = useState<string | null>(null);

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
              {!isConnected && (
                <Badge variant="outline" className="text-[8px] h-4 px-1.5 border-red-500/30 text-red-500 bg-red-500/5 gap-1">
                  <SignalLow className="h-2 w-2" /> OFFLINE
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
        </div>
      </header>
      
      {/* Error Alert */}
      {(error || selectedEpisode?.status === 'failed') && (
        <div className="bg-red-500/10 border-b border-red-500/20 px-6 py-3 flex items-center gap-3 animate-in fade-in slide-in-from-top-1">
          <AlertCircle className="h-5 w-5 text-red-500 shrink-0" />
          <div className="flex-1">
            <p className="text-sm font-bold text-red-500 uppercase tracking-tight">Pipeline Error</p>
            <p className="text-xs text-red-400/80 font-medium">
              {error || "The benchmark generation process encountered an error and could not complete."}
            </p>
          </div>
          <Button 
            variant="ghost" 
            size="sm" 
            className="h-8 text-[10px] font-bold uppercase text-red-400 hover:text-red-300 hover:bg-red-500/10"
            onClick={() => setError(null)}
          >
            Dismiss
          </Button>
        </div>
      )}

      {/* Main Content Area (Grid 9-cols relative to Outlet) */}
      {/* Main Content Area - Resizable Panels */}
      <main className="flex-1 overflow-hidden">
        <ResizablePanelGroup 
          orientation="horizontal" 
          className="h-full w-full"
          onLayoutChanged={(layout: any) => {
            localStorage.setItem('resizable-layout:benchmark-cols', JSON.stringify(layout));
          }}
        >
          {/* Chat Column (Traces) */}
          <ResizablePanel 
            defaultSize={(() => {
              const saved = localStorage.getItem('resizable-layout:benchmark-cols');
              if (saved) {
                try {
                  const layout = JSON.parse(saved);
                  const values = Array.isArray(layout) ? layout : Object.values(layout);
                  return `${values[0] ?? 25}%` as any;
                } catch (e) { return "25%"; }
              }
              return "25%";
            })()} 
            minSize="20%" 
            maxSize="40%"
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
          <ResizablePanel defaultSize="75%" className="min-w-0">
              <ResizablePanelGroup 
                orientation="vertical" 
                className="h-full w-full"
                onLayoutChanged={(layout: any) => {
                  localStorage.setItem('resizable-layout:benchmark-rows', JSON.stringify(layout));
                }}
              >
                  {/* Top: Viewport */}
                  <ResizablePanel 
                    defaultSize={(() => {
                      const saved = localStorage.getItem('resizable-layout:benchmark-rows');
                      if (saved) {
                        try {
                          const layout = JSON.parse(saved);
                          const values = Array.isArray(layout) ? layout : Object.values(layout);
                          return `${values[0] ?? 50}%` as any;
                        } catch (e) { return "50%"; }
                      }
                      return "50%";
                    })()} 
                    minSize="30%"
                  >
                        <div className="h-full relative bg-gradient-to-b from-muted whitespace-nowrap overflow-hidden flex items-center justify-center">
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
                  </ResizablePanel>

                  <ResizableHandle withHandle className="w-full" />

                  {/* Bottom: Artifacts (MJCF / Validation) */}
                  <ResizablePanel defaultSize="50%" minSize="20%">
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
      </main>
    </div>
  );
}