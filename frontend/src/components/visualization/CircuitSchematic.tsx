import React, { useEffect, useState } from 'react';
import { cn } from "../../lib/utils";
import { Badge } from "../ui/badge";
import { useEpisodes } from '../../context/EpisodeContext';

// Import schematic viewer lazily to avoid heavy bundle in other pages
const SchematicViewer = React.lazy(() => import('@tscircuit/schematic-viewer').then(mod => ({ default: mod.SchematicViewer }))) as any;

interface Props {
  className?: string;
}

const CircuitSchematic: React.FC<Props> = ({ className }) => {
  const { selectedEpisode } = useEpisodes();
  const [soup, setSoup] = useState<any[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    if (!selectedEpisode) return;

    const fetchSchematic = async () => {
      try {
        const response = await fetch(`/api/episodes/${selectedEpisode.id}/electronics/schematic`);
        if (response.ok) {
          const data = await response.json();
          setSoup(data);
        }
      } catch (e) {
        console.error("Failed to fetch schematic", e);
      } finally {
        setLoading(false);
      }
    };

    fetchSchematic();
  }, [selectedEpisode]);

  return (
    <div className={cn("bg-slate-950 rounded-xl p-4 border border-slate-800 shadow-2xl relative overflow-hidden group min-h-[400px] flex flex-col", className)}>
      <div className="absolute inset-0 opacity-10 pointer-events-none" 
           style={{ backgroundImage: 'radial-gradient(#3b82f6 1px, transparent 1px)', backgroundSize: '20px 20px' }} 
      />
      
      <div className="flex items-center justify-between mb-4 relative z-10">
          <div className="flex items-center gap-2">
             <div className="size-2 rounded-full bg-blue-500 animate-pulse" />
             <h3 className="text-slate-200 text-[10px] font-black uppercase tracking-widest">Logic Schematic</h3>
          </div>
          <Badge variant="outline" className="text-[8px] border-blue-500/30 text-blue-400 bg-blue-500/5">
              TS_CIRCUIT_V1
          </Badge>
      </div>

      <div className="flex-1 relative z-10 bg-slate-900/50 rounded-lg border border-slate-800 overflow-hidden">
        {loading ? (
            <div className="absolute inset-0 flex items-center justify-center text-[10px] text-muted-foreground uppercase font-bold tracking-widest">
                Loading Netlist...
            </div>
        ) : soup.length > 0 ? (
            <React.Suspense fallback={<div className="flex items-center justify-center h-full text-xs">Initializing Viewer...</div>}>
                <SchematicViewer 
                    soup={soup} 
                    style={{ width: '100%', height: '100%' }}
                />
            </React.Suspense>
        ) : (
            <div className="absolute inset-0 flex items-center justify-center text-[10px] text-muted-foreground uppercase font-bold tracking-widest">
                No Electronics Data
            </div>
        )}
      </div>
    </div>
  );
};

export default CircuitSchematic;
