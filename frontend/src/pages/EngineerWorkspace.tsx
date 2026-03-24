import { useState } from 'react';
import { useEpisodes } from '../context/EpisodeContext';
import { BrainCircuit, Box } from "lucide-react";
import { Button } from "../components/ui/button";
import { Badge } from "../components/ui/badge";
import UnifiedGeneratorView from '../components/workspace/UnifiedGeneratorView';
import { useUISettings } from '../context/UISettingsContext';

function formatMetadataValue(value: unknown, fallback: string): string {
  if (typeof value === 'string' || typeof value === 'number') {
    return String(value);
  }
  if (typeof value === 'boolean') {
    return value ? 'true' : 'false';
  }
  return fallback;
}

export default function EngineerWorkspace() {
  const { selectedEpisode } = useEpisodes();
  const { presentationMode, setPresentationMode } = useUISettings();
  const [resetTrigger, setResetTrigger] = useState(0);
  const metadata = selectedEpisode?.metadata_vars as
    | Record<string, unknown>
    | undefined;
  const cost = formatMetadataValue(metadata?.['cost'], '0.00');
  const weight = formatMetadataValue(metadata?.['weight'], '0');
  const modeLabel = presentationMode ? 'Exit Demo Mode' : 'Enter Demo Mode';

  return (
    <UnifiedGeneratorView
      title="Engineer Workspace"
      subtitle="Agentic CAD Design"
      headerIcon={BrainCircuit}
      storageKeys={{
        cols: 'resizable-layout:workspace-cols',
        rows: 'resizable-layout:workspace-rows'
      }}
      presentationMode={presentationMode}
      headerActions={
        <div className="flex items-center gap-2">
          {presentationMode && (
            <Badge variant="outline" className="text-[9px] h-6 px-3 font-black uppercase tracking-widest border-primary/30 text-primary bg-primary/5">
              Demo Mode
            </Badge>
          )}
          <Button
            variant="outline"
            size="sm"
            className="h-8 px-3 text-[10px] font-bold uppercase tracking-widest"
            onClick={() => setPresentationMode(!presentationMode)}
          >
            {modeLabel}
          </Button>
        </div>
      }
      viewportBadgeText="Live Viewport • Isolated"
      resetTrigger={resetTrigger}
      viewportOverlays={
        <div className="absolute top-4 right-4 z-10 w-40 p-3 space-y-2 bg-background/80 backdrop-blur-md rounded-lg border shadow-sm">
          <div className="flex justify-between items-center">
            <span className="text-[10px] text-muted-foreground uppercase font-black">Est. Cost</span>
            <span className="text-xs font-mono font-bold">${cost}</span>
          </div>
          <div className="flex justify-between items-center border-t border-border/20 pt-2">
            <span className="text-[10px] text-muted-foreground uppercase font-black">Weight</span>
            <span className="text-xs font-mono font-bold">{weight}g</span>
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
