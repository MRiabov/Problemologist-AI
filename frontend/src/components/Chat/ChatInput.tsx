import React, { useState, useRef, type FormEvent, type ChangeEvent, type KeyboardEvent } from 'react';
import { 
  Zap,
  Rocket,
  Plus,
  Mic,
  ArrowRight,
  Square,
  ChevronUp,
  ChevronDown
} from "lucide-react";
import { Button } from "../ui/button";
import { cn } from "../../lib/utils";
import type { TopologyNode } from "../visualization/ModelBrowser";
import type { AssetResponse as Asset } from "../../api/generated/models/AssetResponse";
import type { ContextItem } from "../../context/EpisodeContext";
import { getFileIconInfo } from "../../lib/fileIcons";

interface Suggestion {
  id: string;
  name: string;
  type: 'file' | 'part';
  icon: React.ComponentType<{ className?: string }>;
  original: any;
}

interface ChatInputProps {
  onSendMessage: (prompt: string, metadata: any) => Promise<void>;
  isRunning: boolean;
  onInterrupt: () => void;
  selectedEpisode: { id: string; assets?: Asset[] } | null;
  selectedContext: ContextItem[];
  topologyNodes: TopologyNode[];
  addToContext: (item: ContextItem) => void;
  showObjectives: boolean;
  setShowObjectives: (show: boolean) => void;
}

export function ChatInput({
  onSendMessage,
  isRunning,
  onInterrupt,
  selectedEpisode,
  selectedContext,
  topologyNodes,
  addToContext,
  showObjectives,
  setShowObjectives
}: ChatInputProps) {
  const [prompt, setPrompt] = useState("");
  const [showSuggestions, setShowSuggestions] = useState(false);
  const [suggestions, setSuggestions] = useState<Suggestion[]>([]);
  const [selectedIndex, setSelectedIndex] = useState(0);
  const inputRef = useRef<HTMLTextAreaElement>(null);

  const handleSubmit = async (e?: FormEvent | KeyboardEvent) => {
    if (e) e.preventDefault();
    if (prompt.trim()) {
      // 1. Extract mentions and code references from selectedContext
      const mentions: string[] = [];
      const code_references: any[] = [];
      const selections: any[] = [];

      selectedContext.forEach(item => {
        if (item.type === 'cad') {
          mentions.push(item.label);
        } else if (item.type === 'code') {
          code_references.push({
            file_path: item.metadata?.path,
            start_line: Number(item.metadata?.start || item.metadata?.line),
            end_line: Number(item.metadata?.end || item.metadata?.line)
          });
        }
      });

      // 2. Parse line-range steering from text: @filename:L1-L2
      const steeringRegex = /@([^:\n\s]+):L?(\d+)(?:-L?(\d+))?/g;
      const matches = [...prompt.matchAll(steeringRegex)];
      
      const sessionAssets = selectedEpisode?.assets || [];
      matches.forEach(match => {
        const [, filename, start, end] = match;
        const asset = sessionAssets.find(a => a.s3_path.endsWith(filename) || a.s3_path === filename);
        if (asset) {
            const ref = {
                file_path: asset.s3_path,
                start_line: parseInt(start),
                end_line: end ? parseInt(end) : parseInt(start)
            };
            // Avoid duplicates
            if (!code_references.some(r => r.file_path === ref.file_path && r.start_line === ref.start_line && r.end_line === ref.end_line)) {
                code_references.push(ref);
            }
        }
      });

      // 3. Extract simple mentions from text (e.g. @part_name)
      const mentionRegex = /@([a-zA-Z0-9_\-\.]+)(?![:\d])/g;
      const mentionMatches = [...prompt.matchAll(mentionRegex)];
      mentionMatches.forEach(match => {
        const m = match[1];
        if (!mentions.includes(m)) {
          mentions.push(m);
        }
      });

      const metadata = {
        mentions,
        code_references,
        selections,
        context_items: selectedContext
      };
      
      await onSendMessage(prompt, metadata);
      
      setPrompt("");
      setShowSuggestions(false);
    }
  };

  const handleMention = (value: string, selectionStart: number) => {
    // 1. Line range detection for immediate feedback (Story 4)
    const lineRegex = /@([^:\n\s]+):L?(\d+)(?:-L?(\d+))?/g;
    const matches = [...value.matchAll(lineRegex)];
    
    const sessionAssets = selectedEpisode?.assets || [];
    matches.forEach(match => {
        const [, filename, start, end] = match;
        const asset = sessionAssets.find(a => a.s3_path.endsWith(filename) || a.s3_path === filename);
        if (asset) {
            const contextId = `steer-${asset.id}-${start}-${end || start}`;
            if (!selectedContext.find(i => i.id === contextId)) {
                addToContext({
                    id: contextId,
                    type: 'code',
                    label: `${filename}:${start}${end ? '-' + end : ''}`,
                    metadata: { 
                        path: asset.s3_path, 
                        start: parseInt(start), 
                        end: end ? parseInt(end) : parseInt(start) 
                    }
                });
            }
        }
    });

    // 2. Existing @-mention logic
    const lastAt = value.lastIndexOf('@', selectionStart - 1);
    const filter = value.substring(lastAt + 1, selectionStart);
    
    if (lastAt !== -1 && !/\s/.test(filter)) {
        setShowSuggestions(true);
        
        // Populate suggestions from BOM and Assets
        const items: Suggestion[] = [];
        
        // Add Assets (Files)
        (selectedEpisode?.assets || []).forEach(asset => {
            const name = asset.s3_path.split('/').pop() || asset.s3_path;
            if (name.toLowerCase().includes(filter.toLowerCase())) {
                const iconInfo = getFileIconInfo(name, asset.asset_type);
                items.push({ id: `file-${asset.id}`, name, type: 'file', icon: iconInfo.icon, original: asset });
            }
        });

        // Add BOM Nodes (Parts)
        const collectParts = (nodes: TopologyNode[]) => {
            nodes.forEach(node => {
                if (node.name.toLowerCase().includes(filter.toLowerCase())) {
                    items.push({ id: `part-${node.id}`, name: node.name, type: 'part', icon: Zap, original: node });
                }
                if (node.children) collectParts(node.children);
            });
        };
        collectParts(topologyNodes);

        setSuggestions(items.slice(0, 10)); // Limit to 10
        setSelectedIndex(0);
    } else {
        setShowSuggestions(false);
    }
  };

  const insertSuggestion = (suggestion: Suggestion) => {
    const selectionStart = inputRef.current?.selectionStart || 0;
    const lastAt = prompt.lastIndexOf('@', selectionStart - 1);
    
    const newPrompt = prompt.substring(0, lastAt + 1) + suggestion.name + prompt.substring(selectionStart);
    setPrompt(newPrompt);
    setShowSuggestions(false);
    
    // Auto-add to context for immediate feedback (Story 4)
    if (suggestion.type === 'part') {
        addToContext({
            id: `cad-${suggestion.original.name}`,
            type: 'cad',
            label: suggestion.original.name,
            metadata: { part: suggestion.original.name }
        });
    } else if (suggestion.type === 'file') {
        addToContext({
            id: `code-${suggestion.original.s3_path}`,
            type: 'code',
            label: suggestion.original.s3_path.split('/').pop() || suggestion.original.s3_path,
            metadata: { path: suggestion.original.s3_path }
        });
    }

    setTimeout(() => {
        if (inputRef.current) {
            const newPos = lastAt + 1 + suggestion.name.length;
            inputRef.current.selectionStart = newPos;
            inputRef.current.selectionEnd = newPos;
            inputRef.current.focus();
        }
    }, 0);
  };

  return (
    <div className="bg-muted/30 border border-white/5 rounded-2xl p-3 shadow-sm focus-within:ring-1 focus-within:ring-primary/20 transition-all relative">
        <form onSubmit={(e) => handleSubmit(e)} className="flex flex-col gap-2">
            <textarea 
               id="chat-input"
               placeholder={isRunning ? "Steer the agent..." : "Ask anything (Ctrl+L), @ to mention, / for workflows"}
               className="w-full bg-transparent border-none focus:ring-0 text-[14px] leading-relaxed resize-none min-h-[40px] max-h-48"
               rows={1}
               ref={inputRef}
               value={prompt}
               onChange={(e: ChangeEvent<HTMLTextAreaElement>) => {
                   const val = e.target.value;
                   setPrompt(val);
                   handleMention(val, e.target.selectionStart || 0);
                   e.target.style.height = 'auto';
                   e.target.style.height = e.target.scrollHeight + 'px';
               }}
               onKeyDown={(e: KeyboardEvent<HTMLTextAreaElement>) => {
                   if (showSuggestions && suggestions.length > 0) {
                       if (e.key === 'ArrowDown') {
                           e.preventDefault();
                           setSelectedIndex((prev: number) => (prev + 1) % suggestions.length);
                           return;
                       }
                       if (e.key === 'ArrowUp') {
                           e.preventDefault();
                           setSelectedIndex((prev: number) => (prev - 1 + suggestions.length) % suggestions.length);
                           return;
                       }
                       if (e.key === 'Tab' || e.key === 'Enter') {
                           e.preventDefault();
                           insertSuggestion(suggestions[selectedIndex]);
                           return;
                       }
                       if (e.key === 'Escape') {
                           setShowSuggestions(false);
                           return;
                       }
                   }
                   if (e.key === 'Enter' && !e.shiftKey) {
                        e.preventDefault();
                        handleSubmit(e);
                   }
               }}
            />

            {/* Autocomplete Suggestions */}
            {showSuggestions && suggestions.length > 0 && (
                <div className="absolute bottom-[calc(100%+8px)] left-4 right-4 bg-card border border-border shadow-2xl rounded-xl overflow-hidden z-[100] animate-in slide-in-from-bottom-2 duration-200">
                    <div className="p-2 border-b border-border bg-muted/20">
                        <span className="text-[9px] font-black uppercase tracking-widest text-muted-foreground opacity-70">Suggestions</span>
                    </div>
                    <div className="max-h-48 overflow-y-auto p-1">
                        {suggestions.map((s, idx) => (
                            <button
                                key={s.id}
                                type="button"
                                onClick={(e) => {
                                    e.preventDefault();
                                    insertSuggestion(s);
                                }}
                                className={cn(
                                    "flex items-center gap-2 w-full text-left px-3 py-2 rounded-lg text-[12px] transition-all",
                                    idx === selectedIndex ? "bg-primary/20 text-primary" : "hover:bg-muted/50 text-muted-foreground hover:text-foreground"
                                )}
                            >
                                <s.icon className="h-3.5 w-3.5 shrink-0 opacity-70" />
                                <span className="truncate flex-1">{s.name}</span>
                                <span className="text-[9px] font-mono opacity-30 uppercase">{s.type}</span>
                            </button>
                        ))}
                    </div>
                </div>
            )}
            
            <div className="flex items-center justify-between mt-1">
                <div className="flex items-center gap-1.5">
                    <Button variant="ghost" size="icon" className="h-7 w-7 rounded-lg hover:bg-white/5">
                        <Plus className="h-4 w-4 opacity-40" />
                    </Button>
                    
                    <Button 
                        variant="ghost" 
                        size="sm" 
                        className="h-7 px-2 gap-1 rounded-lg hover:bg-white/5 text-[11px] font-medium text-muted-foreground"
                        onClick={(e) => {
                            e.preventDefault();
                            setShowObjectives(!showObjectives);
                        }}
                    >
                        <Zap className="h-3 w-3 opacity-40" />
                        Planning
                        {showObjectives ? <ChevronDown className="h-3 w-3 ml-0.5 opacity-40" /> : <ChevronUp className="h-3 w-3 ml-0.5 opacity-40" />}
                    </Button>

                    <Button 
                        variant="ghost" 
                        size="sm" 
                        className="h-7 px-2 gap-1 rounded-lg hover:bg-white/5 text-[11px] font-medium text-muted-foreground"
                    >
                        <Rocket className="h-3 w-3 opacity-40" />
                        Claude Opus 4.6
                    </Button>
                </div>

                <div className="flex items-center gap-1.5">
                    <Button variant="ghost" size="icon" className="h-8 w-8 rounded-lg hover:bg-white/5">
                        <Mic className="h-4 w-4 opacity-40" />
                    </Button>
                    
                    <Button 
                       type={isRunning ? "button" : "submit"}
                       size="icon" 
                       disabled={!isRunning && !prompt.trim()}
                       className={cn(
                           "h-8 w-8 rounded-full transition-all flex items-center justify-center",
                           isRunning 
                            ? "bg-red-500/20 text-red-500 hover:bg-red-500/30" 
                            : "bg-foreground text-background hover:opacity-90 disabled:bg-muted/20 disabled:text-muted-foreground"
                       )}
                       aria-label={isRunning ? "Stop Agent" : "Send Message"}
                       onClick={(e) => {
                           if (isRunning && selectedEpisode) {
                               e.preventDefault();
                               onInterrupt();
                           }
                       }}
                    >
                        {isRunning ? <Square className="h-3.5 w-3.5 fill-current" aria-label="Stop Icon" /> : <ArrowRight className="h-4 w-4" aria-label="Send Icon" />}
                    </Button>
                </div>
            </div>
        </form>
    </div>
  );
}
