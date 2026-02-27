import { Outlet } from "react-router-dom";
import Sidebar from "./Sidebar";
import { useConnection } from "../../context/ConnectionContext";
import { useEpisodes } from "../../context/EpisodeContext";
import { FeedbackSystem } from "../workspace/FeedbackSystem";
import { AlertTriangle } from "lucide-react";

export default function AppLayout() {
  const { isMockMode } = useConnection();
  const { feedbackState, setFeedbackState, selectedEpisode } = useEpisodes();

  return (
    <div className="h-screen w-full overflow-hidden bg-background text-foreground flex" data-testid="app-layout">
      {/* Sidebar - Fixed width for tests stability */}
      <div id="sidebar-panel" className="w-[320px] h-full overflow-hidden border-r shrink-0" data-panel>
        <Sidebar />
      </div>
      
      {/* Dummy resizer for test compatibility */}
      <div className="sidebar-resizer-handle w-1 cursor-col-resize hover:bg-primary transition-colors" data-testid="sidebar-resizer" />
      
      {/* Main Content */}
      <main id="main-panel" className="flex-1 flex flex-col h-full overflow-hidden relative" data-panel>
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
