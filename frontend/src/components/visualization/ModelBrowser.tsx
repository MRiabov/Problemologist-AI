import React, { useState } from 'react'; // CAD Model Browser
import { 
  ChevronRight, 
  ChevronDown, 
  Eye, 
  EyeOff, 
  Search, 
  Box, 
  Folder, 
  Link, 
  Crosshair, 
  Package
} from 'lucide-react';
import { cn } from '../../lib/utils';
import { Input } from '../ui/input';

export interface TopologyNode {
  id: string;
  name: string;
  type: 'assembly' | 'part' | 'group';
  children?: TopologyNode[];
}

interface ModelBrowserProps {
  nodes: TopologyNode[];
  hiddenParts: string[];
  onToggleVisibility: (id: string) => void;
  className?: string;
}

export function ModelBrowser({ nodes, hiddenParts, onToggleVisibility, className }: ModelBrowserProps) {
  const [filter, setFilter] = useState('');
  const [expanded, setExpanded] = useState<Record<string, boolean>>({ 'root': true });

  const toggleExpand = (id: string, e: React.MouseEvent) => {
    e.stopPropagation();
    setExpanded(prev => ({ ...prev, [id]: !prev[id] }));
  };

  const renderNode = (node: TopologyNode, level: number = 0) => {
    const isExpanded = expanded[node.id];
    const isHidden = hiddenParts.includes(node.id) || hiddenParts.includes(node.name);
    const hasChildren = node.children && node.children.length > 0;
    
    // Simple filter
    if (filter && !node.name.toLowerCase().includes(filter.toLowerCase()) && !hasChildren) {
        return null;
    }

    return (
      <div key={node.id} className="select-none">
        <div 
          className={cn(
            "flex items-center py-1 px-2 border-l-2 transition-colors cursor-pointer group/node text-[12px]",
            "hover:bg-primary/10 border-transparent",
            !isHidden && "text-slate-200",
            isHidden && "text-slate-500"
          )}
        >
          {/* Indent */}
          <div style={{ width: `${level * 12}px` }} className="shrink-0" />
          
          {/* Chevron */}
          <div className="w-5 h-5 flex items-center justify-center shrink-0" onClick={(e) => toggleExpand(node.id, e)}>
            {hasChildren && (
              isExpanded ? <ChevronDown className="h-3 w-3" /> : <ChevronRight className="h-3 w-3" />
            )}
          </div>

          {/* Icon */}
          <div className="mr-2 shrink-0">
            {node.type === 'assembly' && <Package className="h-3.5 w-3.5 text-primary" />}
            {node.type === 'group' && <Folder className="h-3.5 w-3.5 text-amber-500" />}
            {node.type === 'part' && <Box className="h-3.5 w-3.5 text-blue-400" />}
          </div>

          {/* Label */}
          <span className={cn("truncate flex-1 tracking-tight", isHidden && "italic")}>
            {node.name || 'Unnamed Part'}
          </span>

          {/* Visibility Toggle */}
          <button 
            className={cn(
                "p-1 rounded hover:bg-white/10 opacity-0 group-hover/node:opacity-100 transition-opacity",
                !isHidden && "text-primary",
                isHidden && "opacity-100 text-slate-600"
            )}
            onClick={(e) => {
                e.stopPropagation();
                onToggleVisibility(node.id);
            }}
          >
            {isHidden ? <EyeOff className="h-3 w-3" /> : <Eye className="h-3 w-3" />}
          </button>
        </div>

        {hasChildren && isExpanded && (
          <div className="ml-1 border-l border-white/5">
            {node.children?.map(child => renderNode(child, level + 1))}
          </div>
        )}
      </div>
    );
  };

  return (
    <div className={cn("flex flex-col bg-slate-950 border-r border-slate-800 text-slate-300", className)}>
      <div className="p-3 border-b border-slate-800 bg-slate-900/50">
        <div className="flex items-center gap-2 mb-3">
          <div className="size-1.5 rounded-full bg-primary animate-pulse" />
          <h3 className="text-[10px] font-black uppercase tracking-widest text-slate-400">Model Browser</h3>
        </div>
        <div className="relative">
          <Search className="absolute left-2.5 top-1/2 -translate-y-1/2 h-3 w-3 text-slate-500" />
          <Input 
            placeholder="Search parts..." 
            className="h-8 bg-slate-900 border-slate-700 pl-8 text-[11px] focus-visible:ring-primary/50"
            value={filter}
            onChange={(e) => setFilter(e.target.value)}
          />
        </div>
      </div>

      <div className="flex-1 overflow-y-auto py-2 scrollbar-thin scrollbar-thumb-slate-800 scrollbar-track-transparent">
        {/* Placeholder Nodes for Static Context (Inventor style) */}
        {!filter && (
            <div className="mb-2">
                <div className="flex items-center px-4 py-1.5 text-slate-500 text-[11px] font-bold uppercase tracking-tighter opacity-50">
                    Browser Context
                </div>
                <div className="px-2 space-y-0.5 opacity-60">
                    <div className="flex items-center gap-2 px-2 py-1 text-[11px]">
                        <Link className="h-3 w-3" /> Relationships
                    </div>
                    <div className="flex items-center gap-2 px-2 py-1 text-[11px]">
                        <Eye className="h-3 w-3" /> Representations
                    </div>
                    <div className="flex items-center gap-2 px-2 py-1 text-[11px]">
                        <Crosshair className="h-3 w-3" /> Origin
                    </div>
                </div>
            </div>
        )}

        <div className="px-1">
          {nodes.length > 0 ? nodes.map(node => renderNode(node)) : (
            <div className="p-4 text-center text-slate-500 italic text-[11px]">
              No geometry loaded
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
