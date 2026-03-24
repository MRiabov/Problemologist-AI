import { useEffect, useState } from 'react';
import { Cpu } from "lucide-react";
import { Button } from "../components/ui/button";
import { Badge } from "../components/ui/badge";
import UnifiedGeneratorView from '../components/workspace/UnifiedGeneratorView';
import { useUISettings } from '../context/UISettingsContext';
import { useEpisodes } from '../context/EpisodeContext';

export default function BenchmarkGeneration() {
  const [error, setError] = useState<string | null>(null);
  const { presentationMode, setPresentationMode } = useUISettings();
  const { selectedEpisode, isCreationMode, createNewBenchmark } = useEpisodes();
  const modeLabel = presentationMode ? 'Exit Demo Mode' : 'Enter Demo Mode';
  const benchmarkCreationModeStorageKey = "benchmarkCreationMode";

  useEffect(() => {
    if (localStorage.getItem("selectedEpisodeId") || selectedEpisode || isCreationMode) {
      return;
    }

    if (localStorage.getItem(benchmarkCreationModeStorageKey) === "true") {
      createNewBenchmark(true);
    }
  }, [createNewBenchmark, isCreationMode, selectedEpisode]);

  // Note: Local episode fetching was removed as it seems redundant with EpisodeContext
  // which handles global state for selected episodes/traces.

  return (
    <UnifiedGeneratorView
      title="Benchmark Pipeline"
      subtitle="Mechanical Physics Validation"
      headerIcon={Cpu}
      storageKeys={{
        cols: 'resizable-layout:benchmark-cols',
        rows: 'resizable-layout:benchmark-rows'
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
      viewportBadgeText="Simulation Preview • Multi-Body"
      error={error}
      onDismissError={() => setError(null)}
    />
  );
}
