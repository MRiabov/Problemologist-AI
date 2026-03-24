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
import type { Episode } from "../../api/client";
import type { ContextItem } from "../../context/EpisodeContext";
import { getFileIconInfo } from "../../lib/fileIcons";

interface Suggestion {
  id: string;
  name: string;
  type: 'file' | 'part';
  icon: React.ComponentType<{ className?: string }>;
  original: Asset | TopologyNode;
}

interface ChatInputProps {
  onSendMessage: (prompt: string, metadata: Record<string, unknown>) => Promise<void>;
  isRunning: boolean;
  onInterrupt: () => void;
  selectedEpisode: Pick<Episode, "id" | "assets" | "metadata_vars"> | null;
  selectedContext: ContextItem[];
  topologyNodes: TopologyNode[];
  addToContext: (item: ContextItem) => void;
  showObjectives: boolean;
  setShowObjectives: (show: boolean) => void;
  showPlanningControls?: boolean;
}

const findNodeByName = (nodes: TopologyNode[], name: string): TopologyNode | undefined => {
  for (const node of nodes) {
    if (node.name === name) return node;
    if (node.children) {
      const found = findNodeByName(node.children, name);
      if (found) return found;
    }
  }
  return undefined;
};

const normalizeMentionPath = (value: string): string => value.replace(/^\/+/, "").toLowerCase();

const assetMatchesMention = (assetPath: string, mention: string): boolean => {
  const normalizedAssetPath = normalizeMentionPath(assetPath);
  const normalizedMention = normalizeMentionPath(mention);
  if (!normalizedAssetPath || !normalizedMention) return false;
  if (normalizedAssetPath === normalizedMention) return true;
  if (normalizedAssetPath.endsWith(`/${normalizedMention}`)) return true;
  const assetBasename = normalizedAssetPath.split("/").pop();
  return assetBasename === normalizedMention;
};

export function ChatInput({
  onSendMessage,
  isRunning,
  onInterrupt,
  selectedEpisode,
  selectedContext,
  topologyNodes,
  addToContext,
  showObjectives,
  setShowObjectives,
  showPlanningControls = true
}: ChatInputProps) {
  const [prompt, setPrompt] = useState("");
  const [showSuggestions, setShowSuggestions] = useState(false);
  const [suggestions, setSuggestions] = useState<Suggestion[]>([]);
  const [selectedIndex, setSelectedIndex] = useState(0);
  const inputRef = useRef<HTMLTextAreaElement>(null);
  const scrollRef = useRef<HTMLDivElement>(null);

  const sessionAssets = selectedEpisode?.assets || [];
  const currentPrompt = prompt || inputRef.current?.value || "";
  const contextUsage = selectedEpisode?.metadata_vars?.additional_info?.context_usage;
  const usedChars = Number(contextUsage?.used_tokens);
  const maxChars = Number(contextUsage?.max_tokens);
  const hasContextUsage =
    Number.isFinite(usedChars) && Number.isFinite(maxChars) && maxChars > 0;

  const handleSubmit = async (e?: FormEvent | KeyboardEvent) => {
    if (e) e.preventDefault();
    const submittedPrompt = currentPrompt.trim();
    if (submittedPrompt) {
      // 1. Extract mentions and code references from selectedContext
      const mentions: string[] = [];
      const code_references: Record<string, unknown>[] = [];
      const selections: Record<string, unknown>[] = [];

      selectedContext.forEach(item => {
        if (item.type === 'cad') {
          mentions.push(item.label);
          // Map to structured GeometricSelection if coordinates are present
          if (item.metadata?.center) {
            selections.push({
              level: item.metadata.level || 'PART',
              target_id: item.label,
              center: item.metadata.center,
              normal: item.metadata.normal
            });
          }
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
      const matches = [...submittedPrompt.matchAll(steeringRegex)];
      
      matches.forEach(match => {
        const [, filename, start, end] = match;
        const asset = sessionAssets.find(a => assetMatchesMention(a.s3_path, filename));
        if (asset) {
            const ref = {
                file_path: asset.s3_path,
                start_line: parseInt(start),
                end_line: end ? parseInt(end) : parseInt(start)
            };
            // Avoid duplicates
            if (!code_references.some(r => {
                const refObj = r as { file_path: string; start_line: number; end_line: number };
                return refObj.file_path === ref.file_path && refObj.start_line === ref.start_line && refObj.end_line === ref.end_line;
            })) {
                code_references.push(ref);
            }
        }
      });

      // 3. Extract simple mentions from text (e.g. @part_name)
      const mentionRegex = /@([a-zA-Z0-9_\-.]+)(?![:\d])/g;
      const mentionMatches = [...submittedPrompt.matchAll(mentionRegex)];
      mentionMatches.forEach(match => {
        const name = match[1];
        const part = findNodeByName(topologyNodes, name);
        const asset = sessionAssets.find(a => assetMatchesMention(a.s3_path, name));
        
        // Resolve ID if possible, otherwise use name
        const id = part?.id || asset?.id.toString() || name;
        
        if (!mentions.includes(id)) {
          mentions.push(id);
        }
      });

      const metadata = {
        mentions,
        code_references,
        selections,
        context_items: selectedContext
      };
      
      await onSendMessage(submittedPrompt, metadata);
      
      setPrompt("");
      if (inputRef.current) {
        inputRef.current.value = "";
      }
      setShowSuggestions(false);
    }
  };

  const applyPromptChange = (target: HTMLTextAreaElement) => {
    const val = target.value;
    setPrompt(val);
    handleMention(val, target.selectionStart || 0);
    target.style.height = 'auto';
    target.style.height = target.scrollHeight + 'px';
    handleScroll();
  };

  const handlePromptChange = (event: ChangeEvent<HTMLTextAreaElement>) => {
    applyPromptChange(event.target);
  };

  const handleMention = (value: string, selectionStart: number) => {
    // 1. Line range detection for immediate feedback (Story 4)
    const lineRegex = /@([^:\n\s]+):L?(\d+)(?:-L?(\d+))?/g;
    const matches = [...value.matchAll(lineRegex)];
    
    matches.forEach(match => {
        const [, filename, start, end] = match;
        const asset = sessionAssets.find(a => assetMatchesMention(a.s3_path, filename));
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
    
    const isAtStartOrAfterSpace = lastAt === 0 || (lastAt > 0 && /\s/.test(value[lastAt - 1]));

    if (lastAt !== -1 && isAtStartOrAfterSpace && !/\s/.test(filter)) {
        setShowSuggestions(true);
        
        // Populate suggestions from BOM and Assets
        const items: Suggestion[] = [];
        
        // Add Assets (Files)
        sessionAssets.forEach(asset => {
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
        const node = suggestion.original as TopologyNode;
        addToContext({
            id: `cad-${node.name}`,
            type: 'cad',
            label: node.name,
            metadata: { part: node.name }
        });
    } else if (suggestion.type === 'file') {
        const asset = suggestion.original as Asset;
        addToContext({
            id: `code-${asset.s3_path}`,
            type: 'code',
            label: asset.s3_path.split('/').pop() || asset.s3_path,
            metadata: { path: asset.s3_path }
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

  // Sync scroll between textarea and highlighter
  const handleScroll = () => {
    if (inputRef.current && scrollRef.current) {
        scrollRef.current.scrollTop = inputRef.current.scrollTop;
    }
  };

  return (
    <div className="bg-muted/30 border border-white/5 rounded-2xl p-3 shadow-sm focus-within:ring-1 focus-within:ring-primary/20 transition-all relative">
        <form onSubmit={(e) => handleSubmit(e)} className="flex flex-col gap-2 relative">
            
            {/* Highlighter Layer */}
            <div 
                ref={scrollRef}
                className="absolute inset-0 p-0 pointer-events-none text-[14px] leading-relaxed whitespace-pre-wrap break-words text-transparent overflow-hidden"
                style={{ 
                    padding: '0px',
                    marginTop: '0px',
                    marginLeft: '0px'
                }}
            >
                <div className="w-full px-0 py-0 min-h-[40px]">
                    {(() => {
                        const combinedRegex = /(@([^:\n\s]+)(:L?(\d+)(?:-L?(\d+))?)?)/g;
                        const parts = [];
                        let lastIndex = 0;
                        let match;
                        
                        while ((match = combinedRegex.exec(prompt)) !== null) {
                            const [full, , name, isSteering] = match;
                            parts.push(prompt.substring(lastIndex, match.index));
                            
                            let isValid = false;
                            if (isSteering) {
                                isValid = !!sessionAssets.find(a => assetMatchesMention(a.s3_path, name));
                            } else {
                                const part = findNodeByName(topologyNodes, name);
                                const asset = sessionAssets.find(a => assetMatchesMention(a.s3_path, name));
                                isValid = !!(part || asset);
                            }
                            
                            if (!isValid && name.length > 2) {
                                console.log(`Mention invalid: ${name}. Assets count: ${sessionAssets.length}. Nodes count: ${topologyNodes.length}`);
                                if (sessionAssets.length > 0) {
                                    console.log(`Sample asset: ${sessionAssets[0].s3_path}`);
                                }
                            }

                            parts.push(
                                <span 
                                    key={match.index} 
                                    className={cn(
                                        "rounded px-0.5 font-bold",
                                        isValid 
                                            ? "text-primary bg-primary/10" 
                                            : "text-red-400 bg-red-400/10 underline decoration-dotted underline-offset-2"
                                    )}
                                >
                                    {full}
                                </span>
                            );
                            lastIndex = combinedRegex.lastIndex;
                        }
                        parts.push(prompt.substring(lastIndex));
                        return parts;
                    })()}
                </div>
            </div>

            <textarea 
               id="chat-input"
               placeholder={isRunning ? "Steer the agent..." : "Ask anything (Ctrl+L), @ to mention, / for workflows"}
               className="w-full bg-transparent border-none focus:ring-0 text-[14px] leading-relaxed resize-none min-h-[40px] max-h-48 relative z-10"
               rows={1}
               ref={inputRef}
               value={prompt}
               onScroll={handleScroll}
               onChange={handlePromptChange}
               onInput={(e) => applyPromptChange(e.currentTarget)}
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
                    
                    {showPlanningControls && (
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
                    )}

                    <Button 
                        variant="ghost" 
                        size="sm" 
                        className="h-7 px-2 gap-1 rounded-lg hover:bg-white/5 text-[11px] font-medium text-muted-foreground"
                    >
                        <Rocket className="h-3 w-3 opacity-40" />
                        Claude Opus 4.6
                        {hasContextUsage && (
                            <span
                                data-testid="chatinput-context-usage"
                                className="ml-1.5 text-[10px] font-mono opacity-70"
                            >
                                {`${Math.round(usedChars).toLocaleString()}/${Math.round(maxChars).toLocaleString()}`}
                            </span>
                        )}
                    </Button>
                </div>

                <div className="flex items-center gap-1.5">
                    <Button variant="ghost" size="icon" className="h-8 w-8 rounded-lg hover:bg-white/5">
                        <Mic className="h-4 w-4 opacity-40" />
                    </Button>
                    
                    <Button 
                       type={isRunning ? "button" : "submit"}
                       size="icon" 
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
