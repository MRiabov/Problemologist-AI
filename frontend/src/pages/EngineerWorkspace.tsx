import { useState } from 'react';
import { useEpisodes } from '../context/EpisodeContext';
import { BrainCircuit, Box } from "lucide-react";
import { Button } from "../components/ui/button";
import UnifiedGeneratorView from '../components/workspace/UnifiedGeneratorView';

export default function EngineerWorkspace() {
  const { selectedEpisode } = useEpisodes();
  const [resetTrigger, setResetTrigger] = useState(0);

  return (
    <UnifiedGeneratorView
      title="Engineer Workspace"
      subtitle="Agentic CAD Design"
      headerIcon={BrainCircuit}
      storageKeys={{
        cols: 'resizable-layout:workspace-cols',
        rows: 'resizable-layout:workspace-rows'
      }}
      viewportBadgeText="Live Viewport • Isolated"
      resetTrigger={resetTrigger}
      viewportOverlays={
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
      }
      viewportControls={
        <Button 
          variant="secondary" 
          size="icon" 
          className="h-8 w-8 rounded-full shadow-lg border-primary/20"
          onClick={() => setResetTrigger(prev => prev + 1)}
        >
          <Box className="h-4 w-4" />
        </Button>
      }
    />
  );
}
