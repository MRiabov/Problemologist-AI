import React from 'react';
import { useEpisodes } from '../../context/EpisodeContext';
import { useConnection } from '../../context/ConnectionContext';
import { 
  SignalLow,
  AlertCircle
} from "lucide-react";
import { rebuildModel } from "../../api/client";
import { Button } from "../ui/button";
import { Badge } from "../ui/badge";
import ChatWindow from './ChatWindow';
import ArtifactView from './ArtifactView';
import ConnectionError from '../shared/ConnectionError';
import { 
  ResizableHandle, 
  ResizablePanel, 
  ResizablePanelGroup 
} from "../ui/resizable";
import ModelViewer from '../visualization/ModelViewer';

interface UnifiedGeneratorViewProps {
  title: string;
  subtitle: string;
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
  viewportBadgeText = "Live Viewport",
  viewportOverlays,
  viewportControls,
  resetTrigger,
  error,
  onDismissError
}) => {
  const { 
    selectedEpisode, 
    running
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

  const getAssetUrl = (asset: any) => {
    if (!asset || !selectedEpisode) return null;
    if (asset.s3_path.startsWith('http')) return asset.s3_path;
    return `/api/episodes/${selectedEpisode.id}/assets/${asset.s3_path}`;
  };

  const hasMediaAssets = selectedEpisode?.assets && 
    selectedEpisode.assets.filter(a => a.asset_type === 'video' || a.asset_type === 'image').length > 0;

  const videoAsset = selectedEpisode?.assets?.find(a => a.asset_type === 'video');
  const imageAsset = selectedEpisode?.assets?.find(a => a.asset_type === 'image');
  const modelAssets = selectedEpisode?.assets?.filter(a => a.asset_type === 'stl' || a.asset_type === 'step' || a.asset_type === 'glb') || [];
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
                <Badge variant="outline" className="text-[8px] h-4 px-1.5 border-red-500/30 text-red-500 bg-red-500/5 gap-1">
                  <SignalLow className="h-2 w-2" /> OFFLINE
                </Badge>
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
                  
                  {viewportControls && (
                    <div className="absolute bottom-4 left-4 z-10 flex gap-2">
                       {viewportControls}
                    </div>
                  )}

                  {hasMediaAssets ? (
                    <div className="w-full h-full flex items-center justify-center relative p-8">
                      {!isConnected && <ConnectionError className="absolute inset-0 z-[60]" />}
                      {videoAsset ? (
                        <video 
                          src={getAssetUrl(videoAsset) || ""} 
                          controls 
                          className="max-w-full max-h-full rounded-xl shadow-2xl border-4 border-card z-10"
                        />
                      ) : (
                        <img 
                          src={getAssetUrl(imageAsset) || ""} 
                          className="max-w-full max-h-full object-contain rounded-xl shadow-2xl border-4 border-card z-10"
                        />
                      )}
                    </div>
                  ) : (
                    <div className="w-full h-full relative">
                      <ModelViewer 
                        className="w-full h-full" 
                        assetUrls={modelUrls}
                        isConnected={isConnected}
                        resetTrigger={resetTrigger}
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
                      <div className="absolute top-4 left-1/2 -translate-x-1/2 z-10 pointer-events-none opacity-0 group-hover:opacity-100 transition-opacity">
                        <Badge variant="outline" className="bg-background/50 backdrop-blur-sm text-[10px] uppercase font-bold tracking-widest px-3 py-1 border-primary/20 text-primary/70">
                          {viewportBadgeText}
                        </Badge>
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
};

export default UnifiedGeneratorView;
