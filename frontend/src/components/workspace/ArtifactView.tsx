import { useState, useMemo } from "react";
import { ScrollArea } from "../../components/ui/scroll-area";
import { FileCode, FileText, FileJson, BadgeCheck, Code2, FolderTree, ChevronRight, ChevronDown, File, Play } from "lucide-react";
import { cn } from "../../lib/utils";
import ConnectionError from "../shared/ConnectionError";
import type { AssetResponse } from "../../api/generated/models/AssetResponse";
import { Button } from "../ui/button";
import { runSimulation } from "../../api/client";

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
  const [activeAssetId, setActiveAssetId] = useState<string | 'plan'>(plan ? 'plan' : 'none');
  const [isTreeOpen, setIsTreeOpen] = useState(true);

  // Group assets into a tree structure
  const fileTree = useMemo(() => {
    const tree: any[] = [];
    
    // Add Plan as a special file
    if (plan) {
        tree.push({ name: 'plan.md', type: 'file', icon: FileText, id: 'plan', content: plan });
    }

    // Add real assets
    assets.forEach(asset => {
        let icon = File;
        if (asset.asset_type === 'python') icon = FileCode;
        if (asset.asset_type === 'mjcf') icon = FileJson;
        if (asset.asset_type === 'video' || asset.asset_type === 'image') icon = File; // Could use better icons

        tree.push({
            name: asset.s3_path.split('/').pop() || asset.s3_path,
            type: 'file',
            icon: icon,
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
    if (activeAssetId === 'none' || activeAssetId === 'plan') {
        if (plan) {
            setActiveAssetId('plan');
        } else if (assets.length > 0) {
            setActiveAssetId(assets[0].id.toString());
        }
    }
  }, [plan, assets]);

  const activeAsset = useMemo(() => {
    if (activeAssetId === 'plan') return { name: 'plan.md', content: plan, type: 'plan' };
    const asset = assets.find(a => a.id.toString() === activeAssetId);
    return asset ? { ...asset, name: asset.s3_path.split('/').pop() || asset.s3_path } : null;
  }, [activeAssetId, assets, plan]);

  const renderContent = () => {
    if (!activeAsset) {
        return (
            <div className="flex flex-col items-center justify-center h-64 text-muted-foreground/30 gap-2">
                <Code2 className="h-8 w-8 opacity-20" />
                <span className="text-[10px] uppercase font-bold tracking-widest">No artifact selected</span>
            </div>
        );
    }

    return (
        <div className="flex-1 flex flex-col h-full min-h-0">
            {activeAsset.type === 'plan' && (
                <div className="p-4 border-b bg-green-500/5 flex items-center justify-between shrink-0">
                    <div className="flex flex-col">
                        <span className="text-[10px] font-black uppercase tracking-widest text-green-500/70">Execution Plan</span>
                        <span className="text-[11px] text-muted-foreground">Review and approve the benchmark implementation steps.</span>
                    </div>
                    <Button 
                        size="sm" 
                        className="h-8 bg-green-600 hover:bg-green-700 text-white gap-2 font-bold text-[10px]"
                        onClick={async () => {
                            try {
                                const sessionId = `sim-${Math.random().toString(36).substring(2, 10)}`;
                                await runSimulation(sessionId);
                            } catch (e) {
                                console.error("Failed to start implementation", e);
                            }
                        }}
                    >
                        <Play className="h-3 w-3 fill-current" />
                        START IMPLEMENTATION
                    </Button>
                </div>
            )}
            <ScrollArea className="flex-1">
                <div className="font-mono text-[13px] leading-6 text-gray-300 p-4">
                    {activeAsset.content ? (
                        <pre className="whitespace-pre-wrap">{activeAsset.content}</pre>
                    ) : (
                        <div className="text-muted-foreground/50 italic flex flex-col items-center justify-center h-40 gap-2">
                        <File className="h-8 w-8 opacity-20" />
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
    <div className="flex h-full bg-[#0d1116] border-t border-border overflow-hidden relative">
        {!isConnected && <ConnectionError className="absolute inset-0 z-[100]" />}
        
        {/* Artifact Sidebar (File Tree) */}
        <div className="w-48 border-r border-white/5 flex flex-col bg-muted/5">
            <div className="h-9 px-3 flex items-center border-b border-white/5 bg-white/5">
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
                                {isTreeOpen ? <ChevronDown className="h-3 w-3" /> : <ChevronRight className="h-3 w-3" />}
                                <FolderTree className="h-3 w-3 opacity-70" />
                                {folder.name}
                            </button>
                            {isTreeOpen && folder.children && (
                                <div className="pl-4 space-y-0.5 border-l border-white/5 ml-2.5">
                                    {folder.children.map((file: any) => (
                                         <button
                                            key={file.id}
                                            onClick={() => setActiveAssetId(file.id)}
                                            className={cn(
                                                "flex items-center gap-2 w-full text-left px-2 py-1.5 rounded text-[11px] transition-all",
                                                activeAssetId === file.id 
                                                    ? "bg-primary/20 text-primary font-medium" 
                                                    : "text-muted-foreground hover:bg-white/5 hover:text-foreground"
                                            )}
                                         >
                                            <file.icon className="h-3 w-3" />
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
            <div className="h-9 flex items-center bg-muted/5 border-b border-white/5 px-2 gap-1 overflow-x-auto no-scrollbar">
                <BadgeCheck className={cn(
                    "h-3 w-3 mr-2",
                    activeAssetId === 'plan' ? "text-purple-400" : "text-blue-400"
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

