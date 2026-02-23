import React from 'react';
import { useEpisodes } from '../../context/EpisodeContext';
import { useConnection } from '../../context/ConnectionContext';
import { 
  SignalLow,
  AlertCircle,
  RotateCcw
} from "lucide-react";
import { rebuildModel } from "../../api/client";
import { Button } from "../ui/button";
import { Badge } from "../ui/badge";
import ChatWindow from './ChatWindow';
import ArtifactView from './ArtifactView';
import {
  ResizableHandle,
  ResizablePanel,
  ResizablePanelGroup
} from "../ui/resizable";
import { DesignViewer } from '../visualization/DesignViewer';
import type { AssetResponse } from "../../api/generated/models/AssetResponse";

interface UnifiedGeneratorViewProps {
  title: string;
  subtitle?: string;
  headerIcon: React.ComponentType<{ className?: string }>;
  storageKeys: {
    cols: string;
    rows: string;
  };
  viewportBadgeText?: string;
  viewportOverlays?: React.ReactNode;
  viewportControls?: React.ReactNode;
  resetTrigger?: number;
  error?: string | null;
  onDismissError?: () => void;
}

const UnifiedGeneratorView: React.FC<UnifiedGeneratorViewProps> = ({
  title,
  subtitle,
  headerIcon: HeaderIcon,
  storageKeys,
  viewportBadgeText,
  viewportOverlays,
  viewportControls,
  resetTrigger,
  error,
  onDismissError
}) => {
  const { 
    selectedEpisode, 
    running,
    topologyNodes,
    setTopologyNodes
  } = useEpisodes();
  const { isConnected } = useConnection();

  const getSavedLayout = (key: string, defaultVal: string) => {
    const saved = localStorage.getItem(key);
    if (saved) {
      try {
        const layout = JSON.parse(saved);
        const values = Array.isArray(layout) ? layout : Object.values(layout);
        return `${values[0] ?? defaultVal.replace('%', '')}%` as any;
      } catch (e) { return defaultVal; }
    }
    return defaultVal;
  };

  const getAssetUrl = (asset: AssetResponse) => {
    if (!asset || !selectedEpisode) return null;
    if (asset.s3_path.startsWith('http')) return asset.s3_path;
    return `/api/episodes/${selectedEpisode.id}/assets/${asset.s3_path}`;
  };

  const videoAsset = selectedEpisode?.assets?.find((a: AssetResponse) => a.asset_type === 'video');
  const modelAssets = selectedEpisode?.assets?.filter((a: AssetResponse) => a.asset_type === 'stl' || a.asset_type === 'step' || a.asset_type === 'glb') || [];
  const modelUrls = modelAssets.map(getAssetUrl).filter(Boolean) as string[];

  return (
    <div className="flex flex-col h-full overflow-hidden bg-background">
      {/* Header */}
      <header className="flex shrink-0 items-center justify-between border-b px-6 h-16 bg-card/50 backdrop-blur-sm">
        <div className="flex items-center gap-4">
          <div className="size-10 flex items-center justify-center bg-primary/10 rounded-lg text-primary border border-primary/20">
            <HeaderIcon className="h-6 w-6" />
          </div>
          <div>
                <div className="flex items-center gap-2">
                  <h2 className="text-lg font-bold tracking-tight">{title}</h2>
                  {!isConnected && (
                    <div className="inline-flex items-center rounded-md border px-2.5 py-0.5 text-[8px] h-4 px-1.5 border-red-500/30 text-red-500 bg-red-500/5 gap-1 focus:outline-none focus:ring-2 focus:ring-ring focus:ring-offset-2">
                      <SignalLow className="h-2 w-2" /> OFFLINE
                    </div>
                  )}
                </div>
            <div className="flex items-center gap-2">
              <span className="text-[10px] font-black uppercase tracking-widest text-muted-foreground opacity-70">{subtitle}</span>
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
            <p className="text-sm font-bold text-red-500 uppercase tracking-tight">
              {error ? "Pipeline Error" : "Agent Execution Failed"}
            </p>
            <p className="text-xs text-red-400/80 font-medium">
              {error || "The agent encountered an error. Check the traces for more details."}
            </p>
          </div>
          <Button 
            variant="ghost" 
            size="sm" 
            className="h-8 text-[10px] font-bold uppercase text-red-400 hover:text-red-300 hover:bg-red-500/10"
            onClick={onDismissError}
          >
            {error ? "Dismiss" : "Details Below"}
          </Button>
        </div>
      )}

      {/* Main Content Area */}
      <main className="flex-1 overflow-hidden">
        <ResizablePanelGroup 
          orientation="horizontal" 
          className="h-full w-full"
          data-testid="workspace-resizable-group"
          onLayoutChanged={(layout) => {
            localStorage.setItem(storageKeys.cols, JSON.stringify(layout));
          }}
        >
          {/* Chat Column */}
          <ResizablePanel 
            defaultSize={getSavedLayout(storageKeys.cols, "33.3%")} 
            minSize="20%" 
            maxSize="50%"
            collapsible={true}
            className="overflow-hidden"
          >
            <ChatWindow 
              traces={selectedEpisode?.traces}
              task={selectedEpisode?.task}
              isRunning={running}
              isConnected={isConnected}
              topologyNodes={topologyNodes}
            />
          </ResizablePanel>

          <ResizableHandle withHandle />

          {/* Core View Area */}
          <ResizablePanel defaultSize={75} className="min-w-0">
            <ResizablePanelGroup 
              orientation="vertical" 
              className="h-full w-full"
              onLayoutChanged={(layout) => {
                localStorage.setItem(storageKeys.rows, JSON.stringify(layout));
              }}
            >
              {/* Top: Viewport */}
              <ResizablePanel 
                defaultSize={getSavedLayout(storageKeys.rows, "50%")} 
                minSize="30%"
              >
                <div className="h-full relative bg-gradient-to-b from-muted whitespace-nowrap overflow-hidden flex items-center justify-center group">
                  {viewportOverlays}

                  {viewportBadgeText && (
                    <div className="absolute top-4 left-4 z-10">
                        <Badge variant="secondary" className="bg-background/80 backdrop-blur-md border shadow-sm text-[10px] font-black uppercase tracking-widest py-1 px-3">
                            {viewportBadgeText}
                        </Badge>
                    </div>
                  )}
                  
                  {viewportControls && (
                    <div className="absolute bottom-4 left-4 z-10 flex gap-2">
                       {viewportControls}
                    </div>
                  )}

                  <DesignViewer 
                    modelUrls={modelUrls}
                    videoUrl={videoAsset ? getAssetUrl(videoAsset) : null}
                    heatmapUrls={selectedEpisode?.assets
                      ?.filter(a => (a.s3_path && a.s3_path.includes('stress_')))
                      .map(getAssetUrl)
                      .filter(Boolean) as string[]
                    }
                    isConnected={isConnected}
                    resetTrigger={resetTrigger}
                    topologyNodes={topologyNodes}
                    onTopologyChange={setTopologyNodes}
                    onRebuildModel={async () => {
                        try {
                            await rebuildModel("solution.py");
                            window.location.reload(); 
                        } catch (e) {
                            console.error(e);
                            alert("Failed to rebuild model: " + e);
                        }
                    }}
                  />

                  {modelUrls.length === 0 && !videoAsset && (
                    <div className="absolute inset-0 flex items-center justify-center pointer-events-auto">
                        <div className="bg-slate-900/90 backdrop-blur border border-slate-700 p-4 rounded-xl shadow-2xl flex flex-col items-center gap-3">
                            <h3 className="text-sm font-bold text-slate-200">No Assets Loaded</h3>
                            <Button 
                                size="sm" 
                                variant="outline" 
                                onClick={async () => {
                                    try {
                                        await rebuildModel("solution.py");
                                        window.location.reload(); 
                                    } catch (e) {
                                        console.error(e);
                                        alert("Failed to rebuild model: " + e);
                                    }
                                }}
                                className="w-full gap-2 border-primary/50 text-primary hover:bg-primary/10 hover:text-primary"
                            >
                                <RotateCcw className="h-3 w-3" />
                                Rebuild Assets
                            </Button>
                        </div>
                    </div>
                  )}
                </div>
              </ResizablePanel>

              <ResizableHandle withHandle className="w-full" />

              {/* Bottom: Artifacts */}
              <ResizablePanel defaultSize="50%" minSize="20%">
                <div className="h-full flex-1 overflow-hidden">
                  <ArtifactView 
                    plan={selectedEpisode?.plan || ""}
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
};

export default UnifiedGeneratorView;
