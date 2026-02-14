import { useState } from 'react';
import { useEpisodes } from '../context/EpisodeContext';
import { Cpu } from "lucide-react";
import UnifiedGeneratorView from '../components/workspace/UnifiedGeneratorView';

export default function BenchmarkGeneration() {
  const [error, setError] = useState<string | null>(null);

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
      viewportBadgeText="Simulation Preview • Multi-Body"
      error={error}
      onDismissError={() => setError(null)}
    />
  );
}