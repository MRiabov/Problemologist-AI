import React, { useMemo, useCallback } from 'react';
import { useEffect } from 'react';
import { useEpisodes } from '../../context/EpisodeContext';
import { useConnection } from '../../context/ConnectionContext';
import * as yaml from 'js-yaml';
import { 
  SignalLow,
  AlertCircle,
  RotateCcw
} from "lucide-react";
import { OpenAPI } from '../../api/generated/core/OpenAPI';
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
import { PathUtils } from '../../lib/pathUtils';
import type { AssetResponse } from "../../api/generated/models/AssetResponse";
import { EpisodeStatus } from "../../api/generated/models/EpisodeStatus";
import { EpisodeType } from "../../api/generated/models/EpisodeType";
import {
  getArtifactSelectionDescriptor,
  getDefaultArtifactId,
  getLatestMediaBundle,
} from "./artifactSelection";
import { getEngineerRevisionLineage } from "./RevisionLineageSummary";

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
    episodes,
    selectedEpisode, 
    running,
    topologyNodes,
    setTopologyNodes
  } = useEpisodes();
  const { isConnected } = useConnection();

  const resolvedMediaEpisode = useMemo(() => {
    if (!selectedEpisode) return null;

    const benchmarkId = selectedEpisode.metadata_vars?.benchmark_id?.trim() ?? selectedEpisode.id;
    const latestEngineerEpisode = episodes
      .filter(
        (episode) =>
          episode.metadata_vars?.episode_type === EpisodeType.ENGINEER &&
          episode.metadata_vars?.benchmark_id?.trim() === benchmarkId,
      )
      .filter((episode) => (episode.assets?.length ?? 0) > 0)
      .slice()
      .sort((a, b) => {
        const aTime = new Date(a.created_at).getTime();
        const bTime = new Date(b.created_at).getTime();
        if (aTime !== bTime) {
          return aTime - bTime;
        }
        return String(a.id).localeCompare(String(b.id));
      });

    return latestEngineerEpisode[latestEngineerEpisode.length - 1] ?? selectedEpisode;
  }, [
    episodes,
    selectedEpisode,
    selectedEpisode?.created_at,
    selectedEpisode?.id,
    selectedEpisode?.metadata_vars?.benchmark_id,
    selectedEpisode?.metadata_vars?.episode_type,
  ]);

  const resolvedMediaAssets = resolvedMediaEpisode?.assets ?? selectedEpisode?.assets ?? [];

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

  const getAssetUrl = useCallback((asset: AssetResponse) => {
    const episodeId = resolvedMediaEpisode?.id ?? selectedEpisode?.id ?? null;
    if (!asset || !episodeId) return null;
    if (asset.s3_path.startsWith('http')) return asset.s3_path;
    const path = PathUtils.join(OpenAPI.BASE || '', '/api/episodes', episodeId, 'assets', asset.s3_path);
    return path.startsWith('http') ? path : PathUtils.ensureLeadingSlash(path);
  }, [resolvedMediaEpisode?.id, selectedEpisode?.id]);

  const latestMediaBundle = useMemo(
    () => getLatestMediaBundle(resolvedMediaAssets),
    [resolvedMediaAssets]
  );
  const videoAsset = latestMediaBundle.videoAsset;
  const modelAsset = latestMediaBundle.modelAsset;
  const heatmapAsset = latestMediaBundle.heatmapAsset;
  const defaultSolutionEvidenceAsset = latestMediaBundle.solutionEvidenceAsset;
  const latestMediaBundleKey = useMemo(
    () =>
      [
        videoAsset?.s3_path,
        modelAsset?.s3_path,
        heatmapAsset?.s3_path,
        defaultSolutionEvidenceAsset?.s3_path,
      ]
        .filter(Boolean)
        .join("|"),
    [
      defaultSolutionEvidenceAsset?.s3_path,
      heatmapAsset?.s3_path,
      modelAsset?.s3_path,
      videoAsset?.s3_path,
    ]
  );
  const modelUrls = useMemo(
    () => (modelAsset ? [getAssetUrl(modelAsset)] : []).filter(Boolean) as string[],
    [modelAsset, getAssetUrl]
  );
  const videoUrl = useMemo(
    () => (videoAsset ? getAssetUrl(videoAsset) : null),
    [videoAsset, getAssetUrl]
  );
  const heatmapUrls = useMemo(
    () => (heatmapAsset ? [getAssetUrl(heatmapAsset)] : []).filter(Boolean) as string[],
    [heatmapAsset, getAssetUrl]
  );

  useEffect(() => {
    const urlsToPreload = Array.from(
      new Set(
        [
          ...modelUrls,
          videoUrl,
          ...heatmapUrls,
          defaultSolutionEvidenceAsset ? getAssetUrl(defaultSolutionEvidenceAsset) : null,
        ].filter((url): url is string => !!url),
      ),
    );

    urlsToPreload.forEach((url) => {
      fetch(url, { method: 'GET' }).catch(() => {
        // Preloading is best-effort; the visible viewer still owns the user-facing state.
      });
    });
  }, [defaultSolutionEvidenceAsset, getAssetUrl, heatmapUrls, modelUrls, videoUrl]);

  const defaultArtifactId = useMemo(
    () =>
      getDefaultArtifactId({
        episodeType: resolvedMediaEpisode?.metadata_vars?.episode_type ?? selectedEpisode?.metadata_vars?.episode_type ?? null,
        isBenchmarkRoute: window.location.pathname === '/benchmark',
        plan: resolvedMediaEpisode?.plan ?? selectedEpisode?.plan ?? null,
        assets: resolvedMediaAssets,
      }),
    [
      resolvedMediaAssets,
      resolvedMediaEpisode?.metadata_vars?.episode_type,
      resolvedMediaEpisode?.plan,
      selectedEpisode?.metadata_vars?.episode_type,
      selectedEpisode?.plan,
      window.location.pathname,
    ]
  );
  const terminalSummary = useMemo(() => {
    const metadata = selectedEpisode?.metadata_vars ?? null;
    if (!metadata) return null;
    return {
      detailedStatus: metadata.detailed_status ?? null,
      terminalReason: metadata.terminal_reason ?? null,
      failureClass: metadata.failure_class ?? null,
      validationLogs: metadata.validation_logs ?? [],
      episodeType: metadata.episode_type ?? null,
    };
  }, [selectedEpisode?.metadata_vars]);
  const revisionLineage = useMemo(
    () => getEngineerRevisionLineage(episodes, selectedEpisode),
    [
      episodes,
      selectedEpisode?.id,
      selectedEpisode?.metadata_vars?.benchmark_id,
      selectedEpisode?.metadata_vars?.episode_type,
    ],
  );

  const circuitData = useMemo(() => {
    const assemblyAsset = resolvedMediaAssets.find((a: AssetResponse) => a.s3_path.endsWith('assembly_definition.yaml'));
    if (!assemblyAsset?.content) return null;
    try {
      const data = yaml.load(assemblyAsset.content) as any;
      return data?.electronics ? data : null;
    } catch (e) { 
      console.error("Failed to parse assembly definition for electronics", e);
      return null; 
    }
  }, [resolvedMediaAssets]);

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
      {(error || selectedEpisode?.status === EpisodeStatus.FAILED) && (
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
              traces={selectedEpisode?.traces || []}
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
                className="h-full"
              >
                <div className="h-full w-full relative bg-gradient-to-b from-muted whitespace-nowrap overflow-hidden flex items-center justify-center group" data-testid="viewport-container">
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

                  <div data-testid="unified-debug-info" className="hidden">
                      {JSON.stringify({ 
                          modelUrlsCount: modelUrls.length, 
                          hasVideoAsset: !!videoAsset, 
                          videoAssetType: videoAsset?.asset_type,
                          videoAssetPath: videoAsset?.s3_path ?? null,
                          modelAssetPath: modelAsset?.s3_path ?? null,
                          heatmapAssetPath: heatmapAsset?.s3_path ?? null,
                          solutionEvidenceAssetPath: defaultSolutionEvidenceAsset?.s3_path ?? null,
                          episodeId: selectedEpisode?.id,
                          mediaEpisodeId: resolvedMediaEpisode?.id ?? null,
                          mediaEpisodeStatus: resolvedMediaEpisode?.metadata_vars?.detailed_status || resolvedMediaEpisode?.status || null,
                          episodeStatus: selectedEpisode?.metadata_vars?.detailed_status || selectedEpisode?.status,
                          benchmarkId: selectedEpisode?.metadata_vars?.benchmark_id ?? null,
                          priorEpisodeId: selectedEpisode?.metadata_vars?.prior_episode_id ?? null,
                          isReused: selectedEpisode?.metadata_vars?.is_reused ?? null,
                          revisionCount: revisionLineage.length,
                          revisionHook: selectedEpisode?.metadata_vars?.prior_episode_id
                            ?? selectedEpisode?.metadata_vars?.benchmark_id
                            ?? selectedEpisode?.id
                            ?? null,
                          terminalMetadata: terminalSummary,
                          defaultArtifactId,
                          defaultSolutionEvidenceArtifact: getArtifactSelectionDescriptor(defaultSolutionEvidenceAsset),
                          isRunning: running
                      })}
                  </div>

                  <DesignViewer 
                    modelUrls={modelUrls}
                    videoUrl={videoUrl}
                    heatmapUrls={heatmapUrls}
                    mediaBundleKey={latestMediaBundleKey}
                    circuitData={circuitData}
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
                    <div
                      data-testid="no-assets-overlay"
                      className="absolute inset-0 flex items-center justify-center pointer-events-auto"
                    >
                        <div className="bg-slate-900/90 backdrop-blur border border-slate-700 p-4 rounded-xl shadow-2xl flex flex-col items-center gap-3">
                            <h3 data-testid="no-assets-title" className="text-sm font-bold text-slate-200">No Assets Loaded</h3>
                            <Button 
                                data-testid="rebuild-assets-button"
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

              <ResizableHandle withHandle className="w-full" data-testid="artifact-resizer" />

              {/* Bottom: Artifacts */}
              <ResizablePanel defaultSize="50%" minSize="20%">
                <div className="h-full flex-1 overflow-hidden">
                  <ArtifactView 
                    plan={resolvedMediaEpisode?.plan || selectedEpisode?.plan || ""}
                    assets={resolvedMediaAssets}
                    episodeId={resolvedMediaEpisode?.id ?? selectedEpisode?.id ?? null}
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
