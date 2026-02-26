import { useMemo, useEffect, useState } from "react";
import { ScrollArea } from "../../components/ui/scroll-area";
import { 
  VscCode
} from "react-icons/vsc";
import { 
  Download, 
  ExternalLink, 
  AlertCircle,
  LayoutGrid,
  Search,
  File,
  Play
} from "lucide-react";

import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { vscDarkPlus, vs } from "react-syntax-highlighter/dist/esm/styles/prism";
import * as yaml from "js-yaml";
import { cn } from "../../lib/utils";
import ConnectionError from "../shared/ConnectionError";
import type { AssetResponse } from "../../api/generated/models/AssetResponse";
import type { TraceResponse } from "../../api/generated/models/TraceResponse";
import { useEpisodes, type ContextItem } from "../../context/EpisodeContext";
import { getFileIconInfo as getSharedIconInfo } from "../../lib/fileIcons";
import { detectLanguage } from "../../lib/languageUtils";
import { useTheme } from "../../context/ThemeContext";
import CircuitSchematic from "../visualization/CircuitSchematic";
import CircuitTimeline from "../visualization/CircuitTimeline";
import WireView from "../visualization/WireView";
import { SimulationResults } from "../visualization/SimulationResults";
import { Badge } from "../ui/badge";
import { Button } from "../ui/button";
import { runSimulation } from "../../api/client";
import { AssetType } from "../../api/generated/models/AssetType";
import { TraceType } from "../../api/generated/models/TraceType";
import { EpisodeStatus } from "../../api/generated/models/EpisodeStatus";

interface ArtifactViewProps {
  plan?: string | null;
  assets?: AssetResponse[];
  isConnected?: boolean;
}

export default function ArtifactView({
  plan,
  assets = [],
  isConnected = true
}: ArtifactViewProps) {
  const { 
    selectedEpisode, 
    activeArtifactId, 
    setActiveArtifactId, 
    addToContext, 
    selectedContext,
    running,
    confirmBenchmark
  } = useEpisodes();
  const { theme } = useTheme();
  const [inlineContextLabel, setInlineContextLabel] = useState<string | null>(null);

  const isPlanned = selectedEpisode?.metadata_vars?.detailed_status === 'PLANNED';
  const showExecutionPlan = (selectedEpisode?.plan || isPlanned) && !running && selectedEpisode?.status !== EpisodeStatus.COMPLETED;

  const getAssetUrl = (assetPath: string | undefined) => {
    if (!assetPath || !selectedEpisode) return null;
    if (assetPath.startsWith('http')) return assetPath;
    return `/api/episodes/${selectedEpisode.id}/assets/${assetPath}`;
  };

  const getFileIconInfo = (name: string, type: string) => {
    return getSharedIconInfo(name, type);
  };

  // Group assets into a tree structure
  const fileTree = useMemo(() => {
    const tree: any[] = [];
    
    // Add Plan as a special file
    if (plan) {
        const iconInfo = getFileIconInfo('plan.md', AssetType.MARKDOWN);
        tree.push({ 
          name: 'plan.md', 
          type: 'file', 
          icon: iconInfo.icon, 
          iconColor: iconInfo.color,
          id: 'plan', 
          content: plan 
        });
    }

    // Add real assets
    assets.forEach(asset => {
        const name = asset.s3_path.split('/').pop() || asset.s3_path;
        const iconInfo = getFileIconInfo(name, asset.asset_type);

        tree.push({
            name: name,
            type: 'file',
            icon: iconInfo.icon,
            iconColor: iconInfo.color,
            id: asset.id.toString(),
            content: asset.content,
            asset_type: asset.asset_type,
            s3_path: asset.s3_path
        });
    });

    return [
        { name: 'workspace', type: 'folder', children: tree }
    ];
  }, [plan, assets]);

  // Automatically select first asset if none selected
  useEffect(() => {
    if (!activeArtifactId || activeArtifactId === 'none') {
        if (plan) {
            setActiveArtifactId('plan');
        } else if (assets.length > 0) {
            setActiveArtifactId(assets[0].id.toString());
        }
    }
  }, [plan, assets, activeArtifactId, setActiveArtifactId]);

  const activeAsset = useMemo(() => {
    if (activeArtifactId === 'plan') return { name: 'plan.md', content: plan, asset_type: AssetType.MARKDOWN };
    const asset = assets.find(a => a.id.toString() === activeArtifactId);
    return asset ? { ...asset, name: asset.s3_path.split('/').pop() || asset.s3_path } : null;
  }, [activeArtifactId, assets, plan]);

  const renderContent = () => {
    if (!activeAsset) {
        return (
            <div className="flex flex-col items-center justify-center h-full text-muted-foreground opacity-50">
                <AlertCircle className="h-12 w-12 mb-4" />
                <p>No artifact selected</p>
            </div>
        );
    }

    if (activeAsset.asset_type === AssetType.CIRCUIT_DATA) {
        try {
            const data = typeof activeAsset.content === 'string' ? JSON.parse(activeAsset.content) : activeAsset.content;
            return (
                <div className="flex flex-col gap-6 p-6">
                    <div className="flex items-center justify-between">
                        <h3 className="text-xl font-bold flex items-center gap-2">
                            <VscCode className="text-primary" />
                            Circuit Analysis: {activeAsset.name}
                        </h3>
                    </div>
                    <div className="grid grid-cols-1 xl:grid-cols-2 gap-6">
                        <CircuitSchematic soup={data.electronics} />
                        <WireView 
                            assetUrl={getAssetUrl(assets.find((a: AssetResponse) => a.asset_type === AssetType.GLB || a.asset_type === AssetType.STL)?.s3_path)} 
                            wireRoutes={data.electronics.wiring || []} 
                        />
                    </div>
                </div>
            );
        } catch (e) {
            return <div className="p-4 text-red-500">Failed to parse circuit data</div>;
        }
    }

    if (activeAsset.asset_type === AssetType.TIMELINE) {
        try {
            const data = typeof activeAsset.content === 'string' ? JSON.parse(activeAsset.content) : activeAsset.content;
            return <CircuitTimeline events={data.events} />;
        } catch (e) {
            return <div className="p-4 text-red-500">Failed to parse timeline data</div>;
        }
    }

    // Special rendering for Assembly Definition with Electronics
    if (activeAsset.name === 'assembly_definition.yaml' && activeAsset.content) {
        try {
            const data = yaml.load(activeAsset.content) as any;
            if (data && data.electronics) {
                // Extract timeline events from traces
                const timelineEvents = (selectedEpisode?.traces || [])
                    .filter((t: TraceResponse) => t.trace_type === TraceType.EVENT && t.name === 'circuit_simulation')
                    .map((t: TraceResponse) => ({
                        timestamp: new Date(t.created_at).getTime() / 1000,
                        motor_states: (t as any).metadata?.motor_states || {}
                    }))
                    .sort((a: any, b: any) => a.timestamp - b.timestamp);

                return (
                    <div className="p-6 space-y-8 bg-background min-h-full">
                        <div className="grid grid-cols-1 xl:grid-cols-2 gap-6">
                            <CircuitSchematic />
                            <WireView 
                                assetUrl={getAssetUrl(assets.find(a => a.asset_type === AssetType.GLB || a.asset_type === AssetType.STL)?.s3_path)} 
                                wireRoutes={data.electronics.wiring || []} 
                            />
                        </div>
                        {timelineEvents.length > 0 && (
                            <CircuitTimeline events={timelineEvents} />
                        )}
                        <div className="border-t border-border pt-6">
                            <h3 className="text-slate-200 text-sm font-semibold mb-4">Raw Assembly Definition</h3>
                            <SyntaxHighlighter
                                language="yaml"
                                style={theme === 'dark' ? vscDarkPlus : vs}
                                customStyle={{ margin: 0, padding: '1rem', background: 'transparent', fontSize: '12px' }}
                            >
                                {activeAsset.content}
                            </SyntaxHighlighter>
                        </div>
                    </div>
                );
            }
        } catch (e) {
            console.error("Failed to parse assembly definition for electronics view", e);
        }
    }

    // Special rendering for Simulation Result
    if (activeAsset.name === 'simulation_result.json' && activeAsset.content) {
        try {
            const data = JSON.parse(activeAsset.content);
            return (
                <div className="bg-background min-h-full overflow-y-auto no-scrollbar">
                    <SimulationResults 
                        stressSummaries={data.stress_summaries} 
                        fluidMetrics={data.fluid_metrics} 
                    />
                    <div className="p-6 border-t border-border/50">
                        <h3 className="text-slate-200 text-sm font-semibold mb-4">Raw Simulation Data</h3>
                        <SyntaxHighlighter
                            language="json"
                            style={theme === 'dark' ? vscDarkPlus : vs}
                            customStyle={{ margin: 0, padding: '1rem', background: 'transparent', fontSize: '12px' }}
                        >
                            {activeAsset.content}
                        </SyntaxHighlighter>
                    </div>
                </div>
            );
        } catch (e) {
            console.error("Failed to parse simulation result", e);
        }
    }

    const language = detectLanguage(activeAsset.name, activeAsset.asset_type === AssetType.MJCF ? 'json' : (activeAsset.asset_type || 'text'));
    const addCodeLineContext = (assetPath: string, lineNumber: number) => {
      const label = `${assetPath.split('/').pop()}:${lineNumber}`;
      setInlineContextLabel(label);
      addToContext({
        id: `code-${assetPath}-${lineNumber}`,
        type: 'code',
        label,
        metadata: {
          path: assetPath,
          line: lineNumber
        }
      });
    };

    return (
        <div className="relative h-full overflow-hidden flex flex-col">
            {inlineContextLabel && (
                <div className="px-4 py-2 border-b bg-primary/5">
                    <div
                        data-testid="context-card"
                        className="inline-flex items-center gap-2 px-2 py-1 bg-background border border-border/50 rounded-md shadow-sm"
                    >
                        <span className="text-[11px] font-medium">{inlineContextLabel}</span>
                    </div>
                </div>
            )}
            <div className="flex items-center justify-between px-4 py-2 border-b bg-muted/50">
                <div className="flex items-center gap-2">
                    <span className="text-xs font-medium text-muted-foreground">{activeAsset.name}</span>
                    <Badge variant="outline" className="text-[10px] h-4 px-1">{language}</Badge>
                </div>
                <div className="flex items-center gap-1">
                    <button className="p-1 hover:bg-background rounded transition-colors text-muted-foreground">
                        <Download className="h-3.5 w-3.5" />
                    </button>
                    <button className="p-1 hover:bg-background rounded transition-colors text-muted-foreground">
                        <ExternalLink className="h-3.5 w-3.5" />
                    </button>
                </div>
            </div>
            <div className="flex-1 overflow-auto bg-muted/20">
                <SyntaxHighlighter
                    language={language}
                    style={theme === 'dark' ? vscDarkPlus : vs}
                    customStyle={{
                        margin: 0,
                        padding: '1rem',
                        background: 'transparent',
                        fontSize: '13px',
                        lineHeight: '1.6'
                    }}
                    showLineNumbers={true}
                    wrapLines={true}
                    wrapLongLines={true}
                    lineProps={(lineNumber: number) => {
                        const assetPath = (activeAsset as any).s3_path || activeAsset.name;
                        const isHighlighted = selectedContext.some((item: ContextItem) => 
                            item.type === 'code' && 
                            item.metadata?.path === assetPath && 
                            lineNumber >= (Number(item.metadata?.start) || Number(item.metadata?.line)) && 
                            lineNumber <= (Number(item.metadata?.end) || Number(item.metadata?.line))
                        );
                        
                        return {
                            style: { 
                                display: 'block', 
                                cursor: 'pointer',
                                backgroundColor: isHighlighted ? (theme === 'dark' ? 'rgba(56, 189, 248, 0.15)' : 'rgba(56, 189, 248, 0.1)') : 'transparent',
                                borderLeft: isHighlighted ? '2px solid #38bdf8' : '2px solid transparent'
                            },
                            "data-testid": `code-line-${lineNumber}`,
                            onMouseDown: () => addCodeLineContext(assetPath, lineNumber),
                            onClick: () => {
                                addCodeLineContext(assetPath, lineNumber);
                            }
                        };
                    }}
                >
                    {activeAsset.content || ""}
                </SyntaxHighlighter>
            </div>
        </div>
    );
  };

  return (
    <div className="flex h-full w-full overflow-hidden border rounded-xl bg-card">
        {!isConnected && <ConnectionError className="absolute inset-0 z-[100]" />}
        
        {/* Artifact Sidebar (File Tree) */}
        <div className="w-64 border-r flex flex-col bg-muted/10 shrink-0">
            <div className="p-3 border-b flex items-center justify-between">
                <span className="text-xs font-bold uppercase tracking-wider text-muted-foreground flex items-center gap-2">
                    <LayoutGrid className="h-3.5 w-3.5" /> Resources
                </span>
                <div className="flex items-center gap-2">
                    {showExecutionPlan && (
                        <Button 
                            data-testid="file-explorer-confirm-button"
                            size="sm"
                            variant="ghost"
                            className="h-6 w-6 p-0 bg-primary/10 hover:bg-primary/20 text-primary rounded-md transition-all"
                            title="Confirm & Start Implementation"
                            onClick={async () => {
                                if (!selectedEpisode) return;
                                try {
                                    if (isPlanned) {
                                        await confirmBenchmark(selectedEpisode.id, "");
                                    } else {
                                        const sessionId = `sim-${Math.random().toString(36).substring(2, 10)}`;
                                        await runSimulation(sessionId);
                                    }
                                } catch (e) {
                                    console.error("Failed to start implementation from explorer", e);
                                }
                            }}
                        >
                            <Play className="h-3.5 w-3.5 fill-current" />
                        </Button>
                    )}
                    <Search className="h-3.5 w-3.5 text-muted-foreground" />
                </div>
            </div>
            <ScrollArea className="flex-1 p-2">
                <div className="flex flex-col gap-0.5">
                    {fileTree[0].children.map((item: any) => (
                        <button
                            key={item.id}
                            className={cn(
                                "group flex items-center gap-2 px-2 py-1.5 rounded-md text-[13px] transition-all duration-200",
                                activeArtifactId === item.id 
                                    ? "bg-primary/10 text-primary font-medium shadow-sm" 
                                    : "text-muted-foreground hover:bg-muted hover:text-foreground"
                            )}
                            onClick={() => setActiveArtifactId(item.id)}
                        >
                            <File className={cn("h-4 w-4 shrink-0", activeArtifactId === item.id ? "text-primary" : "text-muted-foreground")} />
                            <span className="truncate">{item.name}</span>
                            {item.asset_type === AssetType.CIRCUIT_DATA && <VscCode className="ml-auto h-3 w-3 opacity-50" />}
                        </button>
                    ))}
                </div>
            </ScrollArea>
        </div>

        {/* Editor Area */}
        <div className="flex-1 min-w-0 bg-background/50">
            {renderContent()}
        </div>
    </div>
  );
}
