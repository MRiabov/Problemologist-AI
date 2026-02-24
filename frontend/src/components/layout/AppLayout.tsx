import { Outlet } from "react-router-dom";
import Sidebar from "./Sidebar";
import { useConnection } from "../../context/ConnectionContext";
import { useEpisodes } from "../../context/EpisodeContext";
import { FeedbackSystem } from "../workspace/FeedbackSystem";
import { AlertTriangle } from "lucide-react";
import { 
  ResizableHandle, 
  ResizablePanel, 
  ResizablePanelGroup 
} from "../ui/resizable";

export default function AppLayout() {
  const { isMockMode } = useConnection();
  const { feedbackState, setFeedbackState, selectedEpisode } = useEpisodes();

  return (
    <div className="h-screen w-full overflow-hidden bg-background text-foreground" data-testid="app-layout">
      <ResizablePanelGroup 
        orientation="horizontal" 
        className="h-full w-full"
        data-testid="main-resizable-group"
        onLayoutChanged={(layout) => {
          localStorage.setItem('resizable-layout:app-sidebar', JSON.stringify(layout));
        }}
      >
        <ResizablePanel 
          defaultSize="25%"
          minSize="15%"
          maxSize="40%"
          collapsible={true}
          className="h-full"
        >
          <div className="h-full overflow-hidden">
            <Sidebar />
          </div>
        </ResizablePanel>
        
        <ResizableHandle withHandle />
        
        <ResizablePanel defaultSize="75%" className="min-w-0">
          <main className="flex flex-col h-full overflow-hidden relative">
            {isMockMode && (
              <div className="bg-amber-500/10 border-b border-amber-500/20 px-4 py-2 flex items-center gap-3 z-50 animate-in slide-in-from-top duration-300">
                <AlertTriangle className="h-4 w-4 text-amber-500" />
                <div className="flex flex-col">
                  <span className="text-[10px] font-black uppercase tracking-wider text-amber-500">Mock LLM Mode Active</span>
                  <span className="text-[10px] text-amber-500/70">Real reasoning traces are disabled. Set IS_INTEGRATION_TEST=false to use real LLM.</span>
                </div>
              </div>
            )}
            <Outlet />
          </main>
        </ResizablePanel>
      </ResizablePanelGroup>

      {/* Global Feedback Modal */}
      {feedbackState && selectedEpisode && (
          <FeedbackSystem 
              episodeId={selectedEpisode.id} 
              traceId={feedbackState.traceId}
              initialScore={feedbackState.score}
              onClose={() => setFeedbackState(null)}
          />
      )}
    </div>
  );
}
