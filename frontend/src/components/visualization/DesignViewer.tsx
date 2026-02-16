import React, { useState } from 'react';
import { Badge } from "../ui/badge";
import { Button } from "../ui/button";
import { 
  Box, 
  PlayCircle, 
  Thermometer, 
  ChevronLeft, 
  ChevronRight,
  Maximize2
} from "lucide-react";
import ModelViewer from './ModelViewer';
import { cn } from "../../lib/utils";

interface DesignViewerProps {
  modelUrls?: string[];
  videoUrl?: string | null;
  heatmapUrls?: string[];
  isConnected?: boolean;
  resetTrigger?: number;
}

type ViewMode = '3d' | 'video' | 'heatmaps';

export const DesignViewer: React.FC<DesignViewerProps> = ({
  modelUrls = [],
  videoUrl,
  heatmapUrls = [],
  isConnected = true,
  resetTrigger = 0
}) => {
  const [viewMode, setViewMode] = useState<ViewMode>(videoUrl ? 'video' : '3d');
  const [activeHeatmapIdx, setActiveHeatmapIdx] = useState(0);

  const hasHeatmaps = heatmapUrls.length > 0;

  return (
    <div className="w-full h-full relative flex flex-col bg-slate-950 overflow-hidden">
      {/* View Mode Switcher */}
      <div className="absolute top-4 left-1/2 -translate-x-1/2 z-30 flex items-center gap-1 p-1 bg-background/40 backdrop-blur-md rounded-full border border-white/10 shadow-2xl">
        <Button
          variant="ghost"
          size="sm"
          className={cn(
            "h-7 px-3 text-[10px] font-bold uppercase tracking-widest rounded-full transition-all",
            viewMode === '3d' ? "bg-primary text-primary-foreground shadow-lg" : "text-muted-foreground hover:text-foreground"
          )}
          onClick={() => setViewMode('3d')}
        >
          <Box className="h-3 w-3 mr-1.5" />
          3D Model
        </Button>
        {videoUrl && (
          <Button
            variant="ghost"
            size="sm"
            className={cn(
              "h-7 px-3 text-[10px] font-bold uppercase tracking-widest rounded-full transition-all",
              viewMode === 'video' ? "bg-primary text-primary-foreground shadow-lg" : "text-muted-foreground hover:text-foreground"
            )}
            onClick={() => setViewMode('video')}
          >
            <PlayCircle className="h-3 w-3 mr-1.5" />
            Simulation
          </Button>
        )}
        {hasHeatmaps && (
          <Button
            variant="ghost"
            size="sm"
            className={cn(
              "h-7 px-3 text-[10px] font-bold uppercase tracking-widest rounded-full transition-all",
              viewMode === 'heatmaps' ? "bg-primary text-primary-foreground shadow-lg" : "text-muted-foreground hover:text-foreground"
            )}
            onClick={() => setViewMode('heatmaps')}
          >
            <Thermometer className="h-3 w-3 mr-1.5" />
            Stress Heatmap
          </Button>
        )}
      </div>

      {/* Main Content Area */}
      <div className="flex-1 relative min-h-0">
        {viewMode === '3d' && (
          <ModelViewer 
            assetUrls={modelUrls} 
            isConnected={isConnected} 
            resetTrigger={resetTrigger}
            className="w-full h-full"
          />
        )}

        {viewMode === 'video' && videoUrl && (
          <div className="w-full h-full flex items-center justify-center bg-black p-8">
            <video 
              src={videoUrl} 
              controls 
              autoPlay
              loop
              className="max-w-full max-h-full rounded-lg shadow-2xl border border-white/5"
            />
          </div>
        )}

        {viewMode === 'heatmaps' && hasHeatmaps && (
          <div className="w-full h-full flex items-center justify-center bg-black relative p-8 group">
            <img 
              src={heatmapUrls[activeHeatmapIdx]} 
              alt={`Stress Heatmap ${activeHeatmapIdx}`}
              className="max-w-full max-h-full object-contain rounded-lg shadow-2xl border border-white/5"
            />
            
            {/* Heatmap Navigation */}
            {heatmapUrls.length > 1 && (
              <>
                <Button
                  variant="ghost"
                  size="icon"
                  className="absolute left-4 top-1/2 -translate-y-1/2 h-10 w-10 rounded-full bg-background/20 backdrop-blur-sm opacity-0 group-hover:opacity-100 transition-opacity"
                  onClick={() => setActiveHeatmapIdx((prev) => (prev - 1 + heatmapUrls.length) % heatmapUrls.length)}
                >
                  <ChevronLeft className="h-6 w-6" />
                </Button>
                <Button
                  variant="ghost"
                  size="icon"
                  className="absolute right-4 top-1/2 -translate-y-1/2 h-10 w-10 rounded-full bg-background/20 backdrop-blur-sm opacity-0 group-hover:opacity-100 transition-opacity"
                  onClick={() => setActiveHeatmapIdx((prev) => (prev + 1) % heatmapUrls.length)}
                >
                  <ChevronRight className="h-6 w-6" />
                </Button>
                
                <div className="absolute bottom-12 left-1/2 -translate-x-1/2 flex gap-1.5">
                  {heatmapUrls.map((_, idx) => (
                    <div 
                      key={idx}
                      className={cn(
                        "h-1 rounded-full transition-all",
                        idx === activeHeatmapIdx ? "w-6 bg-primary" : "w-2 bg-white/20"
                      )}
                    />
                  ))}
                </div>
              </>
            )}

            <div className="absolute top-16 right-4">
               <Badge variant="outline" className="bg-red-500/10 text-red-500 border-red-500/20 text-[10px] font-black uppercase tracking-widest px-3 py-1">
                  Von Mises Stress (Pa)
               </Badge>
            </div>
          </div>
        )}
      </div>

      {/* Footer info badge */}
      <div className="absolute bottom-4 right-4 z-10 pointer-events-none">
         <Badge variant="outline" className="bg-background/20 backdrop-blur-md text-[9px] uppercase font-black tracking-widest px-3 py-1 border-white/10 text-white/50">
            {viewMode === '3d' ? '3D Interactive' : viewMode === 'video' ? 'Time-Step Simulation' : 'Static Analysis'}
         </Badge>
      </div>
    </div>
  );
};
