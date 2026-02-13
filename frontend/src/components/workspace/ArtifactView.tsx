import { useState, useMemo } from "react";
import { ScrollArea } from "../../components/ui/scroll-area";
import { 
  VscMarkdown, 
  VscJson, 
  VscTasklist, 
  VscSettings, 
  VscPlayCircle, 
  VscFileMedia,
  VscChevronDown,
  VscChevronRight,
  VscProject,
  VscCode
} from "react-icons/vsc";
import { SiPython } from "react-icons/si";
import { Prism as SyntaxHighlighter } from "react-syntax-highlighter";
import { vscDarkPlus } from "react-syntax-highlighter/dist/esm/styles/prism";
import { cn } from "../../lib/utils";
import ConnectionError from "../shared/ConnectionError";
import type { AssetResponse } from "../../api/generated/models/AssetResponse";
import { fetchAssetContent, type Asset } from "../../api/client";

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

  const getFileIconInfo = (name: string, type: string) => {
    const fileName = name.toLowerCase();
    
    if (fileName === 'plan.md') return { icon: VscTasklist, color: "#28A745" };
    if (fileName.endsWith('.py') || type === 'python') return { icon: SiPython, color: "#3776AB" };
    if (fileName.endsWith('.md') || type === 'markdown') return { icon: VscMarkdown, color: "#007ACC" };
    if (fileName.endsWith('.json') || fileName.endsWith('.mjcf') || type === 'mjcf') return { icon: VscJson, color: "#FBC02D" };
    if (fileName.endsWith('.yaml') || fileName.endsWith('.yml')) return { icon: VscSettings, color: "#CB2431" };
    if (type === 'video') return { icon: VscPlayCircle, color: "#E44D26" };
    if (type === 'image') return { icon: VscFileMedia, color: "#47A248" };
    
    return { icon: VscCode, color: "#858585" };
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
    if (activeAssetId === 'none' || activeAssetId === 'plan') {
        if (plan) {
            setActiveAssetId('plan');
        } else if (assets.length > 0) {
            setActiveAssetId(assets[0].id.toString());
        }
    }
  }, [plan, assets]);

  const activeAsset = useMemo(() => {
    if (activeAssetId === 'plan') return { name: 'plan.md', content: plan, asset_type: 'markdown' };
    const asset = assets.find(a => a.id.toString() === activeAssetId);
    return asset ? { ...asset, name: asset.s3_path.split('/').pop() || asset.s3_path } : null;
  }, [activeAssetId, assets, plan]);

  const renderContent = () => {
    if (!activeAsset) {
        return (
            <div className="flex flex-col items-center justify-center h-64 text-muted-foreground/30 gap-2">
                <VscCode className="h-8 w-8 opacity-20" />
                <span className="text-[10px] uppercase font-bold tracking-widest">No artifact selected</span>
            </div>
        );
    }

    const language = activeAsset.asset_type === 'mjcf' ? 'json' : (activeAsset.asset_type || 'text');

    return (
        <div className="flex-1 flex flex-col h-full min-h-0">
            {/* Plan header removed - moved to ChatWindow */}
            <ScrollArea className="flex-1">
                <div className="text-[13px] leading-6 p-0">
                    {activeAsset.content ? (
                        <SyntaxHighlighter
                            language={language}
                            style={vscDarkPlus}
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
                                {isTreeOpen ? <VscChevronDown className="h-3 w-3" /> : <VscChevronRight className="h-3 w-3" />}
                                <VscProject className="h-3 w-3 opacity-70" />
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
            <div className="h-9 flex items-center bg-muted/5 border-b border-white/5 px-2 gap-1 overflow-x-auto no-scrollbar">
                <VscCode className={cn(
                    "h-3.5 w-3.5 mr-2",
                    activeAssetId === 'plan' ? "text-green-400" : "text-blue-400"
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

