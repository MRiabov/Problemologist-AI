import { useState } from "react";
import { ScrollArea } from "../../components/ui/scroll-area";
import { FileCode, FileText, FileJson, BadgeCheck, Code2, FolderTree, ChevronRight, ChevronDown } from "lucide-react";
import { cn } from "../../lib/utils";

interface ArtifactViewProps {
  plan?: string | null;
  code?: string; // e.g. impl_build123d.py
  mjcf?: string; // Benchmark XML
  validationResults?: any; // Validation results object
  activeFile?: string; // Allows parent to control active file context
}

export default function ArtifactView({
  plan,
  code,
  mjcf,
  validationResults,
}: ArtifactViewProps) {
  const [activeTab, setActiveTab] = useState("code");
  const [isTreeOpen, setIsTreeOpen] = useState(true);

  // Mock file tree structure based on context
  const fileTree = [
    { name: 'src', type: 'folder', children: [
        { name: 'impl_build123d.py', type: 'file', icon: FileCode, id: 'code' },
        { name: 'benchmark.xml', type: 'file', icon: FileJson, id: 'mjcf' },
    ]},
    { name: 'docs', type: 'folder', children: [
        { name: 'plan.md', type: 'file', icon: FileText, id: 'plan' },
        { name: 'validation.json', type: 'file', icon: BadgeCheck, id: 'validation' },
    ]}
  ];

  const renderContent = () => {
    switch (activeTab) {
      case 'code':
        return (
          <div className="font-mono text-[13px] leading-6 text-gray-300 p-4">
             {code ? (
                 <pre className="whitespace-pre-wrap">{code}</pre>
             ) : (
                 <div className="text-muted-foreground/50 italic flex flex-col items-center justify-center h-40 gap-2">
                    <Code2 className="h-8 w-8 opacity-20" />
                    <span>No Python implementation generated yet.</span>
                 </div>
             )}
          </div>
        );
      case 'mjcf':
        return (
            <div className="font-mono text-[12px] leading-6 text-blue-100 p-4">
               {mjcf ? (
                   <pre className="whitespace-pre-wrap">{mjcf}</pre>
               ) : (
                   <div className="text-muted-foreground/50 italic text-center py-10">
                      Waiting for MJCF generation...
                   </div>
               )}
            </div>
        );
      case 'plan':
            return (
                <div className="prose prose-invert prose-xs max-w-none p-6">
                    {plan ? (
                        <div className="whitespace-pre-wrap font-sans text-sm leading-relaxed text-muted-foreground">
                            {plan}
                        </div>
                    ) : (
                        <div className="text-muted-foreground/50 italic text-center py-10">
                            No active plan.
                        </div>
                    )}
                </div>
            );
      case 'validation':
            return (
                <div className="p-4 font-mono text-xs">
                    {validationResults ? (
                        <pre className="text-green-400 whitespace-pre-wrap">
                            {JSON.stringify(validationResults, null, 2)}
                        </pre>
                    ) : (
                        <div className="text-muted-foreground/50 italic text-center py-10">
                            No validation results available.
                        </div>
                    )}
                </div>
            );
      default:
        return null;
    }
  };

  return (
    <div className="flex h-full bg-[#0d1116] border-t border-border overflow-hidden">
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
                                            onClick={() => setActiveTab(file.id)}
                                            className={cn(
                                                "flex items-center gap-2 w-full text-left px-2 py-1.5 rounded text-[11px] transition-all",
                                                activeTab === file.id 
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
                    activeTab === 'validation' ? "text-green-500" : 
                    activeTab === 'code' ? "text-blue-400" : 
                    activeTab === 'mjcf' ? "text-yellow-500" : "text-purple-400"
                )} />
                <span className="text-[11px] font-mono text-muted-foreground">
                   {activeTab === 'code' ? 'impl_build123d.py' : 
                    activeTab === 'mjcf' ? 'benchmark.xml' : 
                    activeTab === 'plan' ? 'plan.md' : 'validation.json'}
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
