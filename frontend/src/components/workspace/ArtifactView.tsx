import { useState, useMemo } from "react";
import { ScrollArea } from "../../components/ui/scroll-area";
import { 
  VscChevronDown,
  VscChevronRight,
  VscProject,
  VscCode
} from "react-icons/vsc";

import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { vscDarkPlus, vs } from "react-syntax-highlighter/dist/esm/styles/prism";
import yaml from "js-yaml";
import { cn } from "../../lib/utils";
import ConnectionError from "../shared/ConnectionError";
import type { AssetResponse } from "../../api/generated/models/AssetResponse";
import { useEpisodes } from "../../context/EpisodeContext";
import { getFileIconInfo as getSharedIconInfo } from "../../lib/fileIcons";
import { useTheme } from "../../context/ThemeContext";
import CircuitSchematic from "../visualization/CircuitSchematic";
import CircuitTimeline from "../visualization/CircuitTimeline";
import WireView from "../visualization/WireView";
// No unnecessary exports from client

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
  const { activeArtifactId, setActiveArtifactId } = useEpisodes();
  const [isTreeOpen, setIsTreeOpen] = useState(true);

  const getFileIconInfo = (name: string, type: string) => {
    return getSharedIconInfo(name, type);
  };

  // Group assets into a tree structure
  const fileTree = useMemo(() => {
    const tree: any[] = [];
    
    // Add Plan as a special file
    if (plan) {
        const iconInfo = getFileIconInfo('plan.md', 'plan');
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
            asset_type: asset.asset_type
        });
    });

    return [
        { name: 'workspace', type: 'folder', children: tree }
    ];
  }, [plan, assets]);

  // Automatically select first asset if none selected
  useMemo(() => {
    if (!activeArtifactId || activeArtifactId === 'none') {
        if (plan) {
            setActiveArtifactId('plan');
        } else if (assets.length > 0) {
            setActiveArtifactId(assets[0].id.toString());
        }
    }
  }, [plan, assets, activeArtifactId]);

  const activeAsset = useMemo(() => {
    if (activeArtifactId === 'plan') return { name: 'plan.md', content: plan, asset_type: 'markdown' };
    const asset = assets.find(a => a.id.toString() === activeArtifactId);
    return asset ? { ...asset, name: asset.s3_path.split('/').pop() || asset.s3_path } : null;
  }, [activeArtifactId, assets, plan]);

  const { activeEpisode } = useEpisodes();

  const renderContent = () => {
    const { theme } = useTheme();
    if (!activeAsset) {
        return (
            <div className="flex flex-col items-center justify-center h-64 text-muted-foreground/30 gap-2">
                <VscCode className="h-8 w-8 opacity-20" />
                <span className="text-[10px] uppercase font-bold tracking-widest">No artifact selected</span>
            </div>
        );
    }

    // Special rendering for Assembly Definition with Electronics
    if (activeAsset.name === 'preliminary_cost_estimation.yaml' && activeAsset.content) {
        try {
            const data = yaml.load(activeAsset.content) as any;
            if (data && data.electronics) {
                // Extract timeline events from traces
                const timelineEvents = (activeEpisode?.traces || [])
                    .filter(t => t.trace_type === 'event' && t.name === 'circuit_simulation')
                    .map(t => ({
                        timestamp: new Date(t.created_at).getTime() / 1000,
                        motor_states: t.metadata?.motor_states || {}
                    }))
                    .sort((a, b) => a.timestamp - b.timestamp);

                return (
                    <div className="p-6 space-y-8 bg-background min-h-full">
                        <div className="grid grid-cols-1 xl:grid-cols-2 gap-6">
                            <CircuitSchematic electronics={data.electronics} />
                            <WireView 
                                assetUrl={assets.find(a => a.asset_type === 'stl')?.s3_path} 
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

    const language = activeAsset.asset_type === 'mjcf' ? 'json' : (activeAsset.asset_type || 'text');

    return (
        <div className="flex-1 flex flex-col h-full min-h-0 bg-background">
            {/* Plan header removed - moved to ChatWindow */}
            <ScrollArea className="flex-1">
                <div className="text-[13px] leading-6 p-0">
                    {activeAsset.content ? (
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
                            wrapLines={true}
                            wrapLongLines={true}
                        >
                            {activeAsset.content}
                        </SyntaxHighlighter>
                    ) : (
                        <div className="text-muted-foreground/50 italic flex flex-col items-center justify-center h-40 gap-2">
                          <VscCode className="h-8 w-8 opacity-20" />
                          <span>No content available for this asset.</span>
                          { (activeAsset as any).s3_path && <span className="text-[10px] opacity-70">{(activeAsset as any).s3_path}</span> }
                        </div>
                    )}
                </div>
            </ScrollArea>
        </div>
    );
  };

  return (
    <div className="flex h-full bg-background border-t border-border overflow-hidden relative text-foreground">
        {!isConnected && <ConnectionError className="absolute inset-0 z-[100]" />}
        
        {/* Artifact Sidebar (File Tree) */}
        <div className="w-48 border-r border-border/50 flex flex-col bg-muted/5">
            <div className="h-9 px-3 flex items-center border-b border-border/50 bg-muted/10">
                <span className="text-[10px] font-bold text-muted-foreground uppercase tracking-widest">Explorer</span>
            </div>
            <ScrollArea className="flex-1">
                <div className="p-2 space-y-1">
                    {fileTree.map((folder, idx) => (
                        <div key={idx} className="space-y-0.5">
                            <button 
                                onClick={() => setIsTreeOpen(!isTreeOpen)}
                                className="flex items-center gap-1.5 w-full text-left px-2 py-1 text-[11px] font-bold text-muted-foreground hover:text-foreground transition-colors"
                            >
                                {isTreeOpen ? <VscChevronDown className="h-3 w-3" /> : <VscChevronRight className="h-3 w-3" />}
                                <VscProject className="h-3 w-3 opacity-70" />
                                {folder.name}
                            </button>
                            {isTreeOpen && folder.children && (
                                <div className="pl-4 space-y-0.5 border-l border-border/50 ml-2.5">
                                    {folder.children.map((file: any) => (
                                         <button
                                            key={file.id}
                                            onClick={() => setActiveArtifactId(file.id)}
                                            className={cn(
                                                "flex items-center gap-2 w-full text-left px-2 py-1.5 rounded text-[11px] transition-all",
                                                activeArtifactId === file.id 
                                                    ? "bg-primary/20 text-primary font-medium" 
                                                    : "text-muted-foreground hover:bg-muted/50 hover:text-foreground"
                                            )}
                                         >
                                            <file.icon className="h-3.5 w-3.5" style={{ color: file.iconColor }} />
                                            {file.name}
                                         </button>
                                    ))}
                                </div>
                            )}
                        </div>
                    ))}
                </div>
            </ScrollArea>
        </div>

        {/* Editor Area */}
        <div className="flex-1 flex flex-col min-w-0">
            {/* Tabs Header */}
            <div className="h-9 flex items-center bg-muted/5 border-b border-border/50 px-2 gap-1 overflow-x-auto no-scrollbar">
                <VscCode className={cn(
                    "h-3.5 w-3.5 mr-2",
                    activeArtifactId === 'plan' ? "text-green-500" : "text-blue-500"
                )} />
                <span className="text-[11px] font-mono text-muted-foreground">
                   {activeAsset?.name || 'Untitled'}
                </span>
                <div className="flex-1" />
                <span className="text-[9px] text-muted-foreground/30 font-mono">UTF-8</span>
            </div>

            {/* Content Area */}
            <div className="flex-1 flex overflow-hidden relative">
                <ScrollArea className="flex-1">
                    {renderContent()}
                </ScrollArea>
            </div>
        </div>
    </div>
  );
}

